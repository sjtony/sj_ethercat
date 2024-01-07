

/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/** Task period in ns. */
#define PERIOD_NS   (1000000)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state = {};

static ec_slave_config_t *sc_stm_in = NULL;
static ec_slave_config_state_t sc_stm_in_state = {};

static ec_slave_config_t *sc_dig_in = NULL;
static ec_slave_config_state_t sc_dig_in_state = {};

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

#define BusCouplerPos  0, 0
#define DigOutSlavePos 0, 1
#define DigInputSlavePos 0,2
#define CtrlStmSlavePos 0,3
//#define AnaInSlavePos  0, 3
//#define AnaOutSlavePos 0, 4

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL2798 0x00000002, 0x0aee3052
#define Beckhoff_EL1889 0x00000002, 0x07613052
#define Beckhoff_EL7037 0x00000002, 0x1b7d3052
//#define Beckhoff_EL2032 0x00000002, 0x07f03052
#define Beckhoff_EL3152 0x00000002, 0x0c503052
#define Beckhoff_EL3102 0x00000002, 0x0c1e3052
#define Beckhoff_EL4102 0x00000002, 0x10063052

// offsets for PDO entries
static unsigned int off_ana_in_status;
static unsigned int off_ana_in_value;
static unsigned int off_ana_out;
static unsigned int off_dig_out;
static unsigned int off_dig_input;
static unsigned int off_stm_control;
static unsigned int off_stm_velocity;
static unsigned int off_stm_input;

const static ec_pdo_entry_reg_t domain1_regs[] = {
 //   {AnaInSlavePos,  Beckhoff_EL3102, 0x3101, 1, &off_ana_in_status},
 //  {AnaInSlavePos,  Beckhoff_EL3102, 0x3101, 2, &off_ana_in_value},
 //   {AnaOutSlavePos, Beckhoff_EL4102, 0x3001, 1, &off_ana_out},
    {DigInputSlavePos, Beckhoff_EL1889, 0x6000, 1, &off_dig_input},
    {DigOutSlavePos, Beckhoff_EL2798, 0x7000, 1, &off_dig_out},
    {CtrlStmSlavePos, Beckhoff_EL7037, 0x7010, 1, &off_stm_control},
   {CtrlStmSlavePos, Beckhoff_EL7037, 0x7010, 0x21, &off_stm_velocity},
   {CtrlStmSlavePos, Beckhoff_EL7037, 0x6010, 0x01, &off_stm_input},
    {}
};

/* Master 0, Slave 1, "EL2798"
 * Vendor ID:       0x00000002
 * Product code:    0x0aee3052
 * Revision number: 0x00120000
 */



static unsigned int counter = 0;
static unsigned int blink = 0;

/*****************************************************************************/

// Digital out ------------------------

static ec_pdo_entry_info_t el2798_pdo_entries[] = {
    {0x7000, 0x01, 1}, /* Output */
    {0x7010, 0x01, 1}, /* Output */
    {0x7020, 0x01, 1}, /* Output */
    {0x7030, 0x01, 1}, /* Output */
    {0x7040, 0x01, 1}, /* Output */
    {0x7050, 0x01, 1}, /* Output */
    {0x7060, 0x01, 1}, /* Output */
    {0x7070, 0x01, 1}, /* Output */
};

static ec_pdo_info_t el2798_pdos[] = {
    {0x1600, 1, el2798_pdo_entries + 0}, /* Channel 1 */
    {0x1601, 1, el2798_pdo_entries + 1}, /* Channel 2 */
    {0x1602, 1, el2798_pdo_entries + 2}, /* Channel 3 */
    {0x1603, 1, el2798_pdo_entries + 3}, /* Channel 4 */
    {0x1604, 1, el2798_pdo_entries + 4}, /* Channel 5 */
    {0x1605, 1, el2798_pdo_entries + 5}, /* Channel 6 */
    {0x1606, 1, el2798_pdo_entries + 6}, /* Channel 7 */
    {0x1607, 1, el2798_pdo_entries + 7}, /* Channel 8 */
};

static ec_sync_info_t el2798_syncs[] = {
    {0, EC_DIR_OUTPUT, 8, el2798_pdos},
	{1, EC_WD_ENABLE},
    {0xff}
};



/* Master 0, Slave 2, "EL1889"
 * Vendor ID:       0x00000002
 * Product code:    0x07613052
 * Revision number: 0x00130000
 */

ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Input */
    {0x6010, 0x01, 1}, /* Input */
    {0x6020, 0x01, 1}, /* Input */
    {0x6030, 0x01, 1}, /* Input */
    {0x6040, 0x01, 1}, /* Input */
    {0x6050, 0x01, 1}, /* Input */
    {0x6060, 0x01, 1}, /* Input */
    {0x6070, 0x01, 1}, /* Input */
    {0x6080, 0x01, 1}, /* Input */
    {0x6090, 0x01, 1}, /* Input */
    {0x60a0, 0x01, 1}, /* Input */
    {0x60b0, 0x01, 1}, /* Input */
    {0x60c0, 0x01, 1}, /* Input */
    {0x60d0, 0x01, 1}, /* Input */
    {0x60e0, 0x01, 1}, /* Input */
    {0x60f0, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1a00, 1, slave_2_pdo_entries + 0}, /* Channel 1 */
    {0x1a01, 1, slave_2_pdo_entries + 1}, /* Channel 2 */
    {0x1a02, 1, slave_2_pdo_entries + 2}, /* Channel 3 */
    {0x1a03, 1, slave_2_pdo_entries + 3}, /* Channel 4 */
    {0x1a04, 1, slave_2_pdo_entries + 4}, /* Channel 5 */
    {0x1a05, 1, slave_2_pdo_entries + 5}, /* Channel 6 */
    {0x1a06, 1, slave_2_pdo_entries + 6}, /* Channel 7 */
    {0x1a07, 1, slave_2_pdo_entries + 7}, /* Channel 8 */
    {0x1a08, 1, slave_2_pdo_entries + 8}, /* Channel 9 */
    {0x1a09, 1, slave_2_pdo_entries + 9}, /* Channel 10 */
    {0x1a0a, 1, slave_2_pdo_entries + 10}, /* Channel 11 */
    {0x1a0b, 1, slave_2_pdo_entries + 11}, /* Channel 12 */
    {0x1a0c, 1, slave_2_pdo_entries + 12}, /* Channel 13 */
    {0x1a0d, 1, slave_2_pdo_entries + 13}, /* Channel 14 */
    {0x1a0e, 1, slave_2_pdo_entries + 14}, /* Channel 15 */
    {0x1a0f, 1, slave_2_pdo_entries + 15}, /* Channel 16 */
};

ec_sync_info_t slave_2_syncs[] = {
    {0, EC_DIR_INPUT, 16, slave_2_pdos + 0, EC_WD_DISABLE},
    {0xff}
};



/* Master 0, Slave 3, "EL7037"
 * Vendor ID:       0x00000002
 * Product code:    0x1b7d3052
 * Revision number: 0x00130000
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x7000, 0x01, 1}, /* Enable latch C */
    {0x7000, 0x02, 1}, /* Enable latch extern on positive edge */
    {0x7000, 0x03, 1}, /* Set counter */
    {0x7000, 0x04, 1}, /* Enable latch extern on negative edge */
    {0x0000, 0x00, 12}, /* Gap */
    {0x7000, 0x11, 16}, /* Set counter value */
    {0x7010, 0x01, 1}, /* Enable */
    {0x7010, 0x02, 1}, /* Reset */
    {0x7010, 0x03, 1}, /* Reduce torque */
    {0x0000, 0x00, 8}, /* Gap */
    {0x7010, 0x0c, 1}, /* Digital output 1 */
    {0x0000, 0x00, 4}, /* Gap */
    {0x7010, 0x21, 16}, /* Velocity */
    {0x6000, 0x01, 1}, /* Latch C valid */
    {0x6000, 0x02, 1}, /* Latch extern valid */
    {0x6000, 0x03, 1}, /* Set counter done */
    {0x6000, 0x04, 1}, /* Counter underflow */
    {0x6000, 0x05, 1}, /* Counter overflow */
    {0x0000, 0x00, 2}, /* Gap */
    {0x6000, 0x08, 1}, /* Extrapolation stall */
    {0x6000, 0x09, 1}, /* Status of input A */
    {0x6000, 0x0a, 1}, /* Status of input B */
    {0x6000, 0x0b, 1}, /* Status of input C */
    {0x0000, 0x00, 1}, /* Gap */
    {0x6000, 0x0d, 1}, /* Status of extern latch */
    {0x6000, 0x0e, 1}, /* Sync error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x6000, 0x10, 1}, /* TxPDO Toggle */
    {0x6000, 0x11, 16}, /* Counter value */
    {0x6000, 0x12, 16}, /* Latch value */
    {0x6010, 0x01, 1}, /* Ready to enable */
    {0x6010, 0x02, 1}, /* Ready */
    {0x6010, 0x03, 1}, /* Warning */
    {0x6010, 0x04, 1}, /* Error */
    {0x6010, 0x05, 1}, /* Moving positive */
    {0x6010, 0x06, 1}, /* Moving negative */
    {0x6010, 0x07, 1}, /* Torque reduced */
    {0x6010, 0x08, 1}, /* Motor stall */
    {0x0000, 0x00, 3}, /* Gap */
    {0x6010, 0x0c, 1}, /* Digital input 1 */
    {0x6010, 0x0d, 1}, /* Digital input 2 */
    {0x6010, 0x0e, 1}, /* Sync error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x6010, 0x10, 1}, /* TxPDO Toggle */
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1600, 6, slave_3_pdo_entries + 0}, /* ENC RxPDO-Map Control compact */
    {0x1602, 6, slave_3_pdo_entries + 6}, /* STM RxPDO-Map Control */
    {0x1604, 1, slave_3_pdo_entries + 12}, /* STM RxPDO-Map Velocity */
    {0x1a00, 17, slave_3_pdo_entries + 13}, /* ENC TxPDO-Map Status compact */
    {0x1a03, 14, slave_3_pdo_entries + 30}, /* STM TxPDO-Map Status */
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 3, slave_3_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, slave_3_pdos + 3, EC_WD_DISABLE},
    {0xff}
};

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    master_state = ms;

}

/*****************************************************************************/

void check_slave_config_states(void)
{
	printf("slave states checking -- sj \n");
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_dig_in, &s);

    if (s.al_state != sc_dig_in_state.al_state) {
        printf("AnaIn: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_dig_in_state.online) {
        printf("AnaIn: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_dig_in_state.operational) {
        printf("AnaIn: %soperational.\n", s.operational ? "" : "Not ");
    }

    sc_dig_in_state = s;
}

/*****************************************************************************/

void cyclic_task()
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    if (counter) {
        counter--;
    } else { // do this at 1 Hz
        counter = FREQUENCY;

        // calculate new process data
        blink = !blink;

        // check for master state (optional)
        check_master_state();

        // check for slave configuration state(s) (optional)
        check_slave_config_states();
    }

#if 1
    // read process data
    //printf("AnaIn: state %u value %u\n",
    //        EC_READ_U8(domain1_pd + off_ana_in_status),
    //        EC_READ_U16(domain1_pd + off_ana_in_value));
    printf("AnaIn: digital input value %u\n",
            EC_READ_U16(domain1_pd + off_dig_input));
    printf("stm : control value  %u\n",
            EC_READ_U16(domain1_pd + off_stm_control));
    printf("stm : control value  %u\n",
            EC_READ_U16(domain1_pd + off_stm_input));
#endif

#if 1
    // write process data
	printf("blink start -- sj \n");
    EC_WRITE_U8(domain1_pd + off_dig_out, blink ? 0x06 : 0x09);
#endif

    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
    struct timespec wakeup_time;
    int ret = 0;

    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        return -1;
    }

    // Create configuration for el1889 digital input

    if (!(sc_dig_in = ecrt_master_slave_config(
                    master, DigInputSlavePos, Beckhoff_EL1889))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_dig_in, EC_END, slave_2_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // Create configuration for el2798 digital output

    if (!(sc = ecrt_master_slave_config(
                    master, DigOutSlavePos, Beckhoff_EL2798))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc, EC_END, el2798_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // Create configuration for el7037 stepper motor control board

    if (!(sc_stm_in = ecrt_master_slave_config(
                    master, CtrlStmSlavePos, Beckhoff_EL7037))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_stm_in, EC_END, slave_3_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100);
    if (!sc) {
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!  sj\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

    EC_WRITE_U16(domain1_pd + off_stm_velocity, 0x1000);
    EC_WRITE_U8(domain1_pd + off_stm_control, 0x01);
    while (1) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task();

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    return ret;
}

/****************************************************************************/

