#include "eposMotorController.hh"

/****************************************************************************/
/****************************************************************************/

#if USE_ETHERCAT_ECRT_LIB

/*****************************************************************************/

// Functions 
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

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("Application Layer states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

void check_slave_config_states(void)
{
    ec_slave_config_state_t s0,s1,s2,s3;

    ecrt_slave_config_state(sc_epos0, &s0);
    ecrt_slave_config_state(sc_epos1, &s1);
    ecrt_slave_config_state(sc_epos2, &s2);
    // ecrt_slave_config_state(sc_epos3, &s3);

    // s0
    if (s0.al_state != sc_epos0_state.al_state) {
        printf("Epos0: State 0x%02X.\n", s0.al_state);
    }
    if (s0.online != sc_epos0_state.online) {
        printf("Epos0: %s.\n", s0.online ? "online" : "offline");
    }
    if (s0.operational != sc_epos0_state.operational) {
        printf("Epos0: %soperational.\n", s0.operational ? "" : "Not ");
    }
    // s1
    if (s1.al_state != sc_epos1_state.al_state) {
        printf("Epos1: State 0x%02X.\n", s1.al_state);
    }
    if (s1.online != sc_epos1_state.online) {
        printf("Epos1: %s.\n", s1.online ? "online" : "offline");
    }
    if (s1.operational != sc_epos1_state.operational) {
        printf("Epos1: %soperational.\n", s1.operational ? "" : "Not ");
    }
    // s2
    if (s2.al_state != sc_epos2_state.al_state) {
        printf("Epos2: State 0x%02X.\n", s2.al_state);
    }
    if (s2.online != sc_epos2_state.online) {
        printf("Epos2: %s.\n", s2.online ? "online" : "offline");
    }
    if (s2.operational != sc_epos2_state.operational) {
        printf("Epos2: %soperational.\n", s2.operational ? "" : "Not ");
    }
    // // s3
    // if (s3.al_state != sc_epos3_state.al_state) {
    //     printf("Epos3: State 0x%02X.\n", s3.al_state);
    // }
    // if (s3.online != sc_epos3_state.online) {
    //     printf("Epos3: %s.\n", s3.online ? "online" : "offline");
    // }
    // if (s3.operational != sc_epos3_state.operational) {
    //     printf("Epos3: %soperational.\n", s3.operational ? "" : "Not ");
    // }

    sc_epos0_state = s0;
    sc_epos1_state = s1;
    sc_epos2_state = s2;
    // sc_epos3_state = s3;
}

#if SDO_ACCESS

// void read_sdo(void)
// {
//     switch (ecrt_sdo_request_state(sdo_STATUS_WORD)) {
//         case EC_REQUEST_UNUSED: // request was not used yet
//             ecrt_sdo_request_read(sdo_STATUS_WORD); // trigger first read
//             break;
//         case EC_REQUEST_BUSY:
//             printf("Still busy...\n");
//             break;
//         case EC_REQUEST_SUCCESS:
//             printf("SDO value: 0x%04X\n",
//                     EC_READ_U16(ecrt_sdo_request_data(sdo_STATUS_WORD)));
//             ecrt_sdo_request_read(sdo_STATUS_WORD); // trigger next read
//             break;
//         case EC_REQUEST_ERROR:
//             printf("Failed to read SDO!\n");
//             ecrt_sdo_request_read(sdo_STATUS_WORD); // retry reading
//             break;
//     }
// }

// void write_sdo(void)
// {
//     switch (ecrt_sdo_request_state(sdo_CONTROL_WORD)) {
//     case EC_REQUEST_UNUSED: // request was not used yet
//         break;
//     case EC_REQUEST_BUSY:
//         printf("Still busy...\n");
//         break;
//     case EC_REQUEST_SUCCESS:
//         printf("SDO value: 0x%04X\n",
//                 EC_READ_U16(ecrt_sdo_request_data(sdo_STATUS_WORD)));
//         ecrt_sdo_request_read(sdo_STATUS_WORD); // trigger next read
//         break;
//     case EC_REQUEST_ERROR:
//         printf("Failed to read SDO!\n");
//         ecrt_sdo_request_read(sdo_STATUS_WORD); // retry reading
//         break;
//     }
//     // ecrt_sdo_request_t request = ecrt_master_sdo_request(master, 0, CONTROL_WORD,
//     ecrt_sdo_request_write(&request);
// }

#endif

void clearInitialErrors(void)
{
    uint16_t status_word0  =  EC_READ_U16(domain1_pd+off_epos0_STATUS_WORD);// & 0x14FF;
    uint16_t status_word1  =  EC_READ_U16(domain1_pd+off_epos1_STATUS_WORD);// & 0x14FF;    
    uint16_t status_word2  =  EC_READ_U16(domain1_pd+off_epos2_STATUS_WORD);// & 0x14FF;
    // uint16_t status_word3  =  EC_READ_U16(domain1_pd+off_epos3_STATUS_WORD);// & 0x14FF;

    static uint16_t control_word0;
    static uint16_t control_word1;
    static uint16_t control_word2;
    // static uint16_t control_word3;
    
    // epos0
    if ((status_word0& 0x7F)==0)
        control_word0 = 0x00;   
    else if ((status_word0&(1<<3))) // Fault (sts bit 3)
        control_word0= 0x80; // Fault reset (ctr bit 7)
    else if ((status_word0& 0x7F)==0x40) // Switch on disabled (sts bit 6)
        control_word0 = 0x06; // Quick stop + Enable voltage (ctr bits 3,1)
    else if ((status_word0& 0x7F)==0x21) // Quick stop + Ready to switch on (sts bits 5,0)
        control_word0 = 0x0F; // Enable operation + Quick stop 1 + Enable voltage + Switched on (ctr bits 3,2,1,0)
    else
        control_word0 = 0x0F;
    // epos1
    if ((status_word1& 0x7F)==0)
        control_word1 = 0x00;   
    else if ((status_word1&(1<<3))) // Fault (sts bit 3)
        control_word1= 0x80; // Fault reset (ctr bit 7)
    else if ((status_word1& 0x7F)==0x40) // Switch on disabled (sts bit 6)
        control_word1 = 0x06; // Quick stop + Enable voltage (ctr bits 3,1)
    else if ((status_word1& 0x7F)==0x21) // Quick stop + Ready to switch on (sts bits 5,0)
        control_word1 = 0x0F; // Enable operation + Quick stop 1 + Enable voltage + Switched on (ctr bits 3,2,1,0)
    else
        control_word1 = 0x0F;           
    // epos2
    if ((status_word2& 0x7F)==0)
        control_word2 = 0x00;   
    else if ((status_word2&(1<<3))) // Fault (sts bit 3)
        control_word2= 0x80; // Fault reset (ctr bit 7)
    else if ((status_word2& 0x7F)==0x40) // Switch on disabled (sts bit 6)
        control_word2 = 0x06; // Quick stop + Enable voltage (ctr bits 3,1)
    else if ((status_word2& 0x7F)==0x21) // Quick stop + Ready to switch on (sts bits 5,0)
        control_word2 = 0x0F; // Enable operation + Quick stop 1 + Enable voltage + Switched on (ctr bits 3,2,1,0)
    else
        control_word2 = 0x0F;
    // // epos3
    // if ((status_word3& 0x7F)==0)
    //     control_word3 = 0x00;   
    // else if ((status_word3&(1<<3))) // Fault (sts bit 3)
    //     control_word3= 0x80; // Fault reset (ctr bit 7)
    // else if ((status_word3& 0x7F)==0x40) // Switch on disabled (sts bit 6)
    //     control_word3 = 0x06; // Quick stop + Enable voltage (ctr bits 3,1)
    // else if ((status_word3& 0x7F)==0x21) // Quick stop + Ready to switch on (sts bits 5,0)
    //     control_word3 = 0x0F; // Enable operation + Quick stop 1 + Enable voltage + Switched on (ctr bits 3,2,1,0)
    // else
    //     control_word3 = 0x0F;

    EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word0);
    EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word1);
    EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word2);
    // EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word3);
}

setCommandResult setProfilePosition(struct timespec *wakeup_time, unsigned int *counter)
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // check process data state
    check_domain1_state();

    static uint16_t status_word0, control_word0;
    static uint16_t status_word1, control_word1;
    static uint16_t status_word2, control_word2;
    static uint16_t status_word3, control_word3;

    // static uint state = 0;
    if (*counter)
    {
        *counter--;
    }
    else
    { // do this at 1 Hz
        *counter = FREQUENCY/10;
        
        // Epos0
        control_word0   = EC_READ_U16(domain1_pd+off_epos0_CONTROL_WORD);
        status_word0    = EC_READ_U16(domain1_pd+off_epos0_STATUS_WORD);    
        // Epos1
        control_word1   = EC_READ_U16(domain1_pd+off_epos1_CONTROL_WORD);
        status_word1    = EC_READ_U16(domain1_pd+off_epos1_STATUS_WORD);   
        // Epos2
        control_word2   = EC_READ_U16(domain1_pd+off_epos2_CONTROL_WORD);
        status_word2    = EC_READ_U16(domain1_pd+off_epos2_STATUS_WORD);      
        // // Epos3
        // control_word3   = EC_READ_U16(domain1_pd+off_epos3_CONTROL_WORD);
        // status_word3    = EC_READ_U16(domain1_pd+off_epos3_STATUS_WORD);

        //read_sdo();

        // Epos0
        printf("\n----------------------------------------------------------------------------------\n [%.3f] - EPOS 0: Modes of operation: %u Profile velocity: %d Controlword: 0x%x Target position: %d \n", 
            (float)clock()/CLOCKS_PER_SEC*1000,
            EC_READ_U8(domain1_pd+off_epos0_MODES_OF_OPEATION),
            EC_READ_U32(domain1_pd+off_epos0_PROFILE_VELOCITY),
            control_word0,
            EC_READ_S32(domain1_pd+off_epos0_TARGET_POSITION)
            );
        printf(" [%.3f] - EPOS 0: Statusword: 0x%x Position actual value: %d Velocity actual value: %d Modes of operation display: 0x%x \n",
            (float)clock()/CLOCKS_PER_SEC*1000,
            status_word0,
            EC_READ_S32(domain1_pd+off_epos0_POSITION_ACTUAL_VALUE),
            EC_READ_U32(domain1_pd+off_epos0_VELOCITY_ACTUAL_VALUE),
            EC_READ_U8(domain1_pd+off_epos0_MODES_OF_OPERATION_DISPLAY)
            );
        // Epos1
        printf("\n----------------------------------------------------------------------------------\n [%.3f] - EPOS 1: Modes of operation: %u Profile velocity: %d Controlword: 0x%x Target position: %d \n", 
            (float)clock()/CLOCKS_PER_SEC*1000,
            EC_READ_U8(domain1_pd+off_epos1_MODES_OF_OPEATION),
            EC_READ_U32(domain1_pd+off_epos1_PROFILE_VELOCITY),
            control_word1,
            EC_READ_S32(domain1_pd+off_epos1_TARGET_POSITION)
            );
        printf(" [%.3f] - EPOS 1: Statusword: 0x%x Position actual value: %d Velocity actual value: %d Modes of operation display: 0x%x \n",
            (float)clock()/CLOCKS_PER_SEC*1000,
            status_word1,
            EC_READ_S32(domain1_pd+off_epos1_POSITION_ACTUAL_VALUE),
            EC_READ_U32(domain1_pd+off_epos1_VELOCITY_ACTUAL_VALUE),
            EC_READ_U8(domain1_pd+off_epos1_MODES_OF_OPERATION_DISPLAY)
            );
        // Epos2
        printf("\n----------------------------------------------------------------------------------\n [%.3f] - EPOS 2: Modes of operation: %u Profile velocity: %d Controlword: 0x%x Target position: %d \n", 
            (float)clock()/CLOCKS_PER_SEC*1000,
            EC_READ_U8(domain1_pd+off_epos2_MODES_OF_OPEATION),
            EC_READ_U32(domain1_pd+off_epos2_PROFILE_VELOCITY),
            control_word2,
            EC_READ_S32(domain1_pd+off_epos2_TARGET_POSITION)
            );
        printf(" [%.3f] - EPOS 2: Statusword: 0x%x Position actual value: %d Velocity actual value: %d Modes of operation display: 0x%x \n",
            (float)clock()/CLOCKS_PER_SEC*1000,
            status_word2,
            EC_READ_S32(domain1_pd+off_epos2_POSITION_ACTUAL_VALUE),
            EC_READ_U32(domain1_pd+off_epos2_VELOCITY_ACTUAL_VALUE),
            EC_READ_U8(domain1_pd+off_epos2_MODES_OF_OPERATION_DISPLAY)
            );
        // // Epos3
        // printf("\n----------------------------------------------------------------------------------\n [%.3f] - EPOS 3: Modes of operation: %u Profile velocity: %d Controlword: 0x%x Target position: %d \n", 
        //     (float)clock()/CLOCKS_PER_SEC*1000,
        //     EC_READ_U8(domain1_pd+off_epos3_MODES_OF_OPEATION),
        //     EC_READ_U32(domain1_pd+off_epos3_PROFILE_VELOCITY),
        //     control_word3,
        //     EC_READ_S32(domain1_pd+off_epos3_TARGET_POSITION)
        //     );
        // printf(" [%.3f] - EPOS 3: Statusword: 0x%x Position actual value: %d Velocity actual value: %d Modes of operation display: 0x%x \n",
        //     status_word3,
        //     EC_READ_S32(domain1_pd+off_epos3_POSITION_ACTUAL_VALUE),
        //     EC_READ_U32(domain1_pd+off_epos3_VELOCITY_ACTUAL_VALUE),
        //     EC_READ_U8(domain1_pd+off_epos3_MODES_OF_OPERATION_DISPLAY)
        //     );

        // Epos0
        if ((status_word0& 0x7F)==0) // Init state
        {
            printf("\nEPOS 0: 1) Init state\n");
            control_word0 = 0x00;
            EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word0);
        }      
        else if ((status_word0& 0x7F) == 0x08) // Fault (sts bit 3)
        {
            printf("\nEPOS 0: 2) Fault state\n");
            control_word0= 0x80; // Fault reset (ctr bit 7)
            EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word0);
        }  
        else if ((status_word0& 0x7F)==0x40) // Switch on disabled (sts bit 6)
        {    
            printf("\nEPOS 0: 3) Switch on disabled state\n");
            control_word0 = 0x06; // Quick stop + Enable voltage (ctr bits 3,1)
            EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word0);
        }
        else if ((status_word0& 0x7F)==0x21) // Quick stop + Ready to switch on (sts bits 5,0)
        {
            control_word0 = 0x0F; // Enable operation + Quick stop 1 + Enable voltage + Switched on (ctr bits 3,2,1,0)
            EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word0);
            //control_word0 = 0x0F; // Enable operation + Quick stop 1 + Enable voltage + Switched on (ctr bits 3,2,1,0)
            // if((status_word0&0x145E) == 0x1016) // Set point acklowledge + Target not reached + No fault + Voltage enabled + Operation enabled + Switched on 
            // {
            printf("\nEPOS 0: 4) Normal operation - Quick stop + Ready to switch on\n");
            EC_WRITE_U8(domain1_pd+off_epos0_MODES_OF_OPEATION, 0x01); // printf("1) Modes of operation = Profile position mode [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            EC_WRITE_U32(domain1_pd+off_epos0_PROFILE_VELOCITY, 2000); // printf("2) Profile velocity = 2000 rpm [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            EC_WRITE_S32(domain1_pd+off_epos0_TARGET_POSITION, 75000); // printf("4) Target position = 10000 tics [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            control_word0 = 0x1F; // PPM
            EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word0); // printf("5) Controlword0 = Absolute position [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            printf("EPOS 0: Finished setting profile position [%lu ms]\n",wakeup_time->tv_nsec/1000000);

            // }    
            if ((status_word0&0x145E) == 0x1416)  // Target reached
            {
                printf("EPOS 0: Target reached\n");
                control_word0 = 0x06; // Shutdown
                EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word0); // printf("1) Controlword0 = Shutdown [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            }
            // else
            // {
            //     printf("Not normal operation\n");           
            //     control_word0 = 0x06; // Shutdown
            //     EC_WRITE_U16(domain1_pd+off_epos0_CONTROL_WORD, control_word0); // printf("1) Controlword0 = Shutdown [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            // }
        }
        else
            control_word0 = 0x0F;

        // Epos1
        if ((status_word1& 0x7F)==0) // Init state
        {
            printf("\nEPOS 1: 1) Init state\n");
            control_word1 = 0x00;
            EC_WRITE_U16(domain1_pd+off_epos1_CONTROL_WORD, control_word1);
        }      
        else if ((status_word1& 0x7F) == 0x08) // Fault (sts bit 3)
        {
            printf("\nEPOS 1: 2) Fault state\n");
            control_word1= 0x80; // Fault reset (ctr bit 7)
            EC_WRITE_U16(domain1_pd+off_epos1_CONTROL_WORD, control_word1);
        }  
        else if ((status_word1& 0x7F)==0x40) // Switch on disabled (sts bit 6)
        {    
            printf("\nEPOS 1: 3) Switch on disabled state\n");
            control_word1 = 0x06; // Quick stop + Enable voltage (ctr bits 3,1)
            EC_WRITE_U16(domain1_pd+off_epos1_CONTROL_WORD, control_word1);
        }
        else if ((status_word1& 0x7F)==0x21) // Quick stop + Ready to switch on (sts bits 5,0)
        {
            control_word1 = 0x0F; // Enable operation + Quick stop 1 + Enable voltage + Switched on (ctr bits 3,2,1,0)
            EC_WRITE_U16(domain1_pd+off_epos1_CONTROL_WORD, control_word1);
            //control_word1 = 0x0F; // Enable operation + Quick stop 1 + Enable voltage + Switched on (ctr bits 3,2,1,0)
            // if((status_word1&0x145E) == 0x1016) // Set point acklowledge + Target not reached + No fault + Voltage enabled + Operation enabled + Switched on 
            // {
            printf("\nEPOS 1: 4) Normal operation - Quick stop + Ready to switch on\n");
            EC_WRITE_U8(domain1_pd+off_epos1_MODES_OF_OPEATION, 0x01); // printf("1) Modes of operation = Profile position mode [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            EC_WRITE_U32(domain1_pd+off_epos1_PROFILE_VELOCITY, 2000); // printf("2) Profile velocity = 2000 rpm [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            EC_WRITE_S32(domain1_pd+off_epos1_TARGET_POSITION, 75000); // printf("4) Target position = 10000 tics [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            control_word1 = 0x1F; // PPM
            EC_WRITE_U16(domain1_pd+off_epos1_CONTROL_WORD, control_word1); // printf("5) Controlword1 = Absolute position [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            printf("EPOS 1: Finished setting profile position [%lu ms]\n",wakeup_time->tv_nsec/1000000);

            // }    
            if ((status_word1&0x145E) == 0x1416)  // Target reached
            {
                printf("EPOS 1: Target reached\n");
                control_word1 = 0x06; // Shutdown
                EC_WRITE_U16(domain1_pd+off_epos1_CONTROL_WORD, control_word1); // printf("1) Controlword1 = Shutdown [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            }
            // else
            // {
            //     printf("Not normal operation\n");           
            //     control_word1 = 0x06; // Shutdown
            //     EC_WRITE_U16(domain1_pd+off_epos1_CONTROL_WORD, control_word1); // printf("1) Controlword1 = Shutdown [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            // }
        }
        else
            control_word1 = 0x0F;

        // Epos2
        if ((status_word2& 0x7F)==0) // Init state
        {
            printf("\nEPOS 2: 1) Init state\n");
            control_word2 = 0x00;
            EC_WRITE_U16(domain1_pd+off_epos2_CONTROL_WORD, control_word2);
        }      
        else if ((status_word2& 0x7F) == 0x08) // Fault (sts bit 3)
        {
            printf("\nEPOS 2: 2) Fault state\n");
            control_word2= 0x80; // Fault reset (ctr bit 7)
            EC_WRITE_U16(domain1_pd+off_epos2_CONTROL_WORD, control_word2);
        }  
        else if ((status_word2& 0x7F)==0x40) // Switch on disabled (sts bit 6)
        {    
            printf("\nEPOS 2: 3) Switch on disabled state\n");
            control_word2 = 0x06; // Quick stop + Enable voltage (ctr bits 3,1)
            EC_WRITE_U16(domain1_pd+off_epos2_CONTROL_WORD, control_word2);
        }
        else if ((status_word2& 0x7F)==0x21) // Quick stop + Ready to switch on (sts bits 5,0)
        {
            control_word2 = 0x0F; // Enable operation + Quick stop 1 + Enable voltage + Switched on (ctr bits 3,2,1,0)
            EC_WRITE_U16(domain1_pd+off_epos2_CONTROL_WORD, control_word2);
            //control_word2 = 0x0F; // Enable operation + Quick stop 1 + Enable voltage + Switched on (ctr bits 3,2,1,0)
            // if((status_word2&0x145E) == 0x1016) // Set point acklowledge + Target not reached + No fault + Voltage enabled + Operation enabled + Switched on 
            // {
            printf("\nEPOS 2: 4) Normal operation - Quick stop + Ready to switch on\n");
            EC_WRITE_U8(domain1_pd+off_epos2_MODES_OF_OPEATION, 0x01); // printf("1) Modes of operation = Profile position mode [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            EC_WRITE_U32(domain1_pd+off_epos2_PROFILE_VELOCITY, 2000); // printf("2) Profile velocity = 2000 rpm [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            EC_WRITE_S32(domain1_pd+off_epos2_TARGET_POSITION, 75000); // printf("4) Target position = 10000 tics [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            control_word2 = 0x1F; // PPM
            EC_WRITE_U16(domain1_pd+off_epos2_CONTROL_WORD, control_word2); // printf("5) Controlword1 = Absolute position [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            printf("EPOS 2: Finished setting profile position [%lu ms]\n",wakeup_time->tv_nsec/1000000);

            // }    
            if ((status_word2&0x145E) == 0x1416)  // Target reached
            {
                printf("EPOS 2: Target reached\n");
                control_word2 = 0x06; // Shutdown
                EC_WRITE_U16(domain1_pd+off_epos2_CONTROL_WORD, control_word2); // printf("1) Controlword2 = Shutdown [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            }
            // else
            // {
            //     printf("Not normal operation\n");           
            //     control_word2 = 0x06; // Shutdown
            //     EC_WRITE_U16(domain1_pd+off_epos2_CONTROL_WORD, control_word2); // printf("1) Controlword2 = Shutdown [%lu ms]\n",wakeup_time->tv_nsec/1000000);
            // }
        }
        else
            control_word2 = 0x0F;

        // Repeat for Epos3
        // Epos3
    }

    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
	
	return setCommandResult::SUCCESS_SET_C;
}

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

initMotorsResult initMotors(void)
{
	ec_slave_config_t *sc;
    int ret = 0;

    master = ecrt_request_master(0);
    if (!master) {
		fprintf(stderr, "Failed to request master0.\n");
        return initMotorsResult::ERROR_INIT_M;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
		fprintf(stderr, "Failed to create domain.\n");
        return initMotorsResult::ERROR_INIT_M;
    }

    // Epos0
    if (!(sc_epos0 = ecrt_master_slave_config(master, Epos0Pos, EPOS4))) {
        fprintf(stderr, "EPOS 0: Failed to get slave configuration.\n");
        return initMotorsResult::ERROR_INIT_M;
    }
    // Epos1
    if (!(sc_epos1 = ecrt_master_slave_config(master, Epos1Pos, EPOS4))) {
        fprintf(stderr, "EPOS 1: Failed to get slave configuration.\n");
        return initMotorsResult::ERROR_INIT_M;
    }
    // Epos2
	if (!(sc_epos2 = ecrt_master_slave_config(master, Epos2Pos, EPOS4))) {
        fprintf(stderr, "EPOS 2: Failed to get slave configuration.\n");
        return initMotorsResult::ERROR_INIT_M;
    }
	// Repeat for Epos3
    // Epos3

    // Epos0
    printf("EPOS 0: Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_epos0, EC_END, epos0_syncs)) {
        fprintf(stderr, "EPOS 0: Failed to configure PDOs.\n");
        return initMotorsResult::ERROR_INIT_M;
    }

    // Epos1
    printf("EPOS 1: Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_epos1, EC_END, epos1_syncs)) {
        fprintf(stderr, "EPOS 1: Failed to configure PDOs.\n");
        return initMotorsResult::ERROR_INIT_M;
    }

    // Epos2
    printf("EPOS 2: Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_epos2, EC_END, epos2_syncs)) {
        fprintf(stderr, "EPOS 1: Failed to configure PDOs.\n");
        return initMotorsResult::ERROR_INIT_M;
    }

    // Repeat for Epos3
    // Epos3

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed.\n");
        return initMotorsResult::ERROR_INIT_M;
    }

    #if SDO_ACCESS

        // printf("Configuring SDOs...\n");

        // if (!(sdo_STATUS_WORD = ecrt_slave_config_create_sdo_request(sc_epos0, STATUS_WORD, 2))) {
        //     fprintf(stderr, "Failed to create SDO request.\n");
        // }
        // ecrt_sdo_request_timeout(sdo_STATUS_WORD, 500); // ms

        // if (!(sdo_CONTROL_WORD = ecrt_slave_config_create_sdo_request(sc_epos0, CONTROL_WORD, 2))) {
        //     fprintf(stderr, "Failed to create SDO request.\n");
        // }
        // ecrt_sdo_request_timeout(sdo_CONTROL_WORD, 500); // ms

    #endif

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "Failed to activate master.\n");
        return initMotorsResult::ERROR_INIT_M;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return initMotorsResult::ERROR_INIT_M;
    }
	
	return initMotorsResult::SUCCESS_INIT_M;
}

runProgramResult runProgram(void)
{
    int ret = 0;
	struct timespec wakeup_time;

	// Initialize motors in the EtherCAT network 
	static initMotorsResult initMotorsRes = initMotors();
	if(initMotorsRes != initMotorsResult::SUCCESS_INIT_M)
	{
		fprintf(stderr, "Error during initialzation of motors in EtherCAT network.\n");
		return runProgramResult::ERROR_INIT_MOTORS;
	}

	// Set command result returns the status of the operating commands (PPM, PVM, CPM, CVM or CTM)
	static setCommandResult setCommandRes = setCommandResult::ERROR_SET_C;

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

    while (1) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        setCommandRes = setProfilePosition(&wakeup_time, &counter);

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

	if(setCommandRes != setCommandResult::SUCCESS_SET_C)
	{
		fprintf(stderr, "Error during set command.\n");
		return runProgramResult::ERROR_SET_COMMAND;
	}
	else
		return runProgramResult::RUNNING;
}

#endif

/****************************************************************************/
/****************************************************************************/

#if USE_USB_EPOS_CMD_LIB

// Functions
void initEposMotorController()
{
    mParams.nodeID = DEFAULT_EPOS_NODE_ID;
    mParams.deviceName = DEFAULT_EPOS_DEVICE_NAME;
    mParams.protocolStackName = DEFAULT_EPOS_PROTOCOL_STACK_NAME;
    mParams.interfaceName = DEFAULT_EPOS_INTERFACE_NAME;
    mParams.portName = DEFAULT_EPOS_PORT_NAME;
    mParams.baudrate = DEFAULT_EPOS_BAUDRATE;
    mParams.p_deviceHandle = DEFAULT_EPOS_DEVICE_HANDLE;
    mParams.position = DEFAULT_EPOS_POSITION;
    mParams.velocity = DEFAULT_EPOS_VELOCITY;
    mParams.current = DEFAULT_EPOS_CURRENT;
}

void initEposMotorController(MotorControllerParameters& a_mParams)
{
    mParams = a_mParams;
}

void LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	std::cerr <<"EPOS: " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
}

void LogInfo(std::string message)
{
	std::cout << message << std::endl;
}

int Init()
{
    int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
    
    // Open the connection to the device
    mParams.p_deviceHandle = NULL;
    mParams.p_deviceHandle = OpenDevice(&ulErrorCode);
    
    if(ulErrorCode!=MMC_SUCCESS)
    {
        LogError("OpenDevice", lResult, ulErrorCode);
        return lResult;
    }

    // Prepare the device
	if((lResult = PrepareDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("PrepareDevice", lResult, ulErrorCode);
        return lResult;
    }

    return lResult;
}

HANDLE OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;
	void* _pKeyHandle = NULL;
	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, mParams.deviceName.c_str());
	strcpy(pProtocolStackName, mParams.protocolStackName.c_str());
	strcpy(pInterfaceName, mParams.interfaceName.c_str());
	strcpy(pPortName, mParams.portName.c_str());

	LogInfo("Open device...");

	_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(_pKeyHandle, mParams.baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(mParams.baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		_pKeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return _pKeyHandle;

}

int PrepareDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(mParams.p_deviceHandle, mParams.nodeID, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			std::stringstream msg;
			msg << "clear fault, node = '" << mParams.nodeID << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(mParams.p_deviceHandle, mParams.nodeID, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(mParams.p_deviceHandle, mParams.nodeID, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(mParams.p_deviceHandle, mParams.nodeID, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(mParams.p_deviceHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int setPosition(unsigned int & p_rlErrorCode, long a_position)
{
    mParams.position = a_position;
	
    int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set profile position mode, node = " << mParams.nodeID<<std::endl;

	LogInfo(msg.str());

	if(VCS_ActivateProfilePositionMode(mParams.p_deviceHandle, mParams.nodeID, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{
		msg << "move to position = " << mParams.position << ", node = " << mParams.nodeID;
		LogInfo(msg.str());
		if(VCS_MoveToPosition(mParams.p_deviceHandle, mParams.nodeID, mParams.position, 0, 1, &p_rlErrorCode) == 0)
		{
			LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
			lResult = MMC_FAILED;
		}
		sleep(2);
		if(lResult == MMC_SUCCESS)
		{
			LogInfo("halt position movement");
			if(VCS_HaltPositionMovement(mParams.p_deviceHandle, mParams.nodeID, &p_rlErrorCode) == 0)
			{
				LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
			}
		}
	}
	return lResult;
}

long getPosition()
{
    return mParams.position;
}

int setVelocity(unsigned int & p_rlErrorCode, long a_velocity)
{
    mParams.velocity = a_velocity;
	
    int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set profile velocity mode, node = " << mParams.nodeID<<std::endl;

	LogInfo(msg.str());

	if(VCS_ActivateProfileVelocityMode(mParams.p_deviceHandle, mParams.nodeID, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{
		msg << "move with target velocity = " << mParams.velocity << " rpm, node = " << mParams.nodeID;
		LogInfo(msg.str());
		if(VCS_MoveWithVelocity(mParams.p_deviceHandle, mParams.nodeID, mParams.velocity, &p_rlErrorCode) == 0)
		{
			LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
			lResult = MMC_FAILED;
        }
		sleep(1);
		if(lResult == MMC_SUCCESS)
		{
			LogInfo("halt velocity movement");
			if(VCS_HaltVelocityMovement(mParams.p_deviceHandle, mParams.nodeID, &p_rlErrorCode) == 0)
			{
				LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
                lResult = MMC_FAILED;
			}
		}
	}
	return lResult;
}

long getVelocity()
{
    return mParams.velocity;
}

int setCurrent(long a_current)
{
    mParams.current = a_current;
    return 1;
}

long getCurrent()
{
    return mParams.current;
}

#endif

/****************************************************************************/
/****************************************************************************/