 /** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the Esmacat slave project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "my_app.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/**
 * @brief Identifies the actual Esmacat slave sequence in the EtherCAT communication chain.
 */
void my_app::assign_slave_sequence(){
    // tell the master what type of slave is at which point in the chain
    assign_esmacat_slave_index(&ecat_epos,0);
}

/**
 * @brief Configure your Esmacat slave.
 * Link Esmacat slave object with the actual Esmacat slave in the EtherCAT communication chain.
 * Functions beginning with 'configure_slave' must only be executed in this function
 */
void my_app::configure_slaves(){

    // add initialization code here
    // Functions starting with "configure_slave" work only in configure_slaves() function
}

/** @brief Initialization that needs to happen on the first iteration of the loop
 */
void my_app::init()
{
    // Reset previous Errorcode
    ecat_epos.reset_fault();

    // Set Operation in Cyclic Synchronous Torque Mode
    ecat_epos.set_mode_operation(10);

}

/**
 * @brief Executes functions at the defined loop rate
 */
void my_app::loop(){
    // add functions below that are to be executed at the loop rate

    if (loop_cnt < 500)
    {
        ecat_epos.stop_motor();
   }
    else
    {
        ecat_epos.start_motor();

        // Compute setpoint
        double setpoint = 50*sin(2*3.1415*elapsed_time_ms/1000.0);
        ecat_epos.set_target_torque(static_cast<int16_t>(setpoint));
        if (loop_cnt%100 == 0)
        {
            cout << "Setpoint: " << setpoint << " Position: " << ecat_epos.get_position() << " deg" << endl;
        }
    }


    if (loop_cnt > 1000000)
    {
        ecat_epos.stop_motor();

        if(loop_cnt > 1000500)
        {
            printf("\nWARNING: Real time loop rate was exceeded %d times out of a total of %ld\n", get_app_error_counter(), loop_cnt);
            stop();
        }
    }
}
