/*
 * USB Initialization Module
 *
 * written by Ken Fodero
 * BioRobotics Lab, University of Washington
 * ken@ee.washington.edu
 *
 * Modified by Hawkeye King
 *
 */

#include "USB_init.h"
#include <string.h>
#include <vector>
#include <map>
#include <dirent.h>
#include <iostream>
#include <stdio.h>
#include <ros/console.h>

//Four device files for connection to four boards
#define BRL_USB_DEV_DIR     "/dev/"
#define BOARD_FILE_STR      "brl_usb"   /// Device file. xx is the place holder of the serial number. restricted to 2 digits for now
#define NUM_BOARDS 99
#define BOARD_FILE_STR_LEN sizeof(BOARD_FILE_STR)
#define BRL_RESET_BOARD     10

// Keep board information
std::vector<int> boardFile;
std::map<int,int> boardFPs;

extern USBStruct USBBoards;
extern int NUM_MECH;

using namespace std;

/**
* List directory contents matching BOARD_FILE_STR
*/
int getdir (string dir, vector<string> &files)
{
    DIR *dp;
    struct dirent *dirp;

    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        if ( strstr(dirp->d_name, BOARD_FILE_STR) ){
            files.push_back(string(dirp->d_name));
        }
    }
    closedir(dp);
    return 0;
}

int get_board_id_from_filename(string s)
{
    string tmp = s.substr(7,s.length()-7);
	return atoi(tmp.c_str());
}

int write_zeros_to_board(int boardid)
{
    short int tmp = DAC_OFFSET;
    unsigned char buffer_out[MAX_OUT_LENGTH];

    buffer_out[0]= DAC;        //Type of USB packet
    buffer_out[1]= MAX_DOF_PER_MECH; //Number of DAC channels
    for(int i = 0; i < MAX_DOF_PER_MECH; i++) {
        buffer_out[2*i+2] = (char)tmp;
        buffer_out[2*i+3] = (char)tmp>>8;
    }
    buffer_out[OUT_LENGTH-1] = 0x00;

    //Write the packet to the USB Driver
    if(usb_write(boardid, &buffer_out, OUT_LENGTH )!= OUT_LENGTH){
        return -USB_WRITE_ERROR;
    }

    return 0;

}

/**
 * USBInit() - initialize the USB modules
 *
 * input: none
 *
 * Set the number of starting USB boards
 * Make Sure a USB board is attached
 * Store data about the boards attached
 *
 * \return status of initialization: 0 if no USB board found, USB_INIT_ERROR if an error was encountered or # of boards if initialized successfully
 *
 */
int USBInit(struct device *device0)
{
    char buf[10]; //buffer to be used for clearing usb read buffers
    string boardStr;
    int boardid = 0;
    int okboards = 0;

    // Get list of files in dev dir
    vector<string> files = vector<string>();
    getdir(BRL_USB_DEV_DIR, files);

    ROS_INFO("  Found board files::");
    for (unsigned int i = 0;i < files.size();i++) {
        ROS_INFO("    %s", files[i].c_str());
    }

    //Initialize all active USB Boards
    //Open and reset available boards
    USBBoards.activeAtStart=0;
    for (uint i=0;i<files.size();i++)
    {
        boardStr = BRL_USB_DEV_DIR;
        boardStr += files[i];
        boardid = get_board_id_from_filename(files[i]);

        /// Open usb dev
        int tmp_fileHandle = open(boardStr.c_str(), O_RDWR|O_NONBLOCK);    //Is NONBLOCK mode required??// open board chardev
        if (tmp_fileHandle <=0 )
        {
            perror("ERROR: coultn't open board");
            errno=0;
            continue; //Failed to open board, move to next one
        }

        /// Setup usb dev.  ioctl() performs an initialization in driver.
        if ( ioctl(tmp_fileHandle, BRL_RESET_BOARD) != 0)
        {
            ROS_ERROR("ERROR: ioctl error opening board %s", boardStr.c_str());
            errno = 0;
        }
        log_msg ("Boards Opened");


        device0->mech[i].type = 0;
        /// Set mechanism type Green or Gold surgical robot
        if (boardid == GREEN_ARM_SERIAL)
        {
            okboards++;
            log_msg("  Green Arm on board #%d.",boardid);
            device0->mech[i].type = GREEN_ARM;
        }
        else if (boardid == GOLD_ARM_SERIAL)
        {
            okboards++;
            log_msg("  Gold Arm on board #%d.",boardid);
            device0->mech[i].type = GOLD_ARM;
        }
        else
        {
            log_msg("*** WARNING: USB BOARD #%d NOT CONNECTED TO MECH (update defines?).",boardid);
        }

        log_msg ("okboards = %d", okboards);

        /// Store usb dev parameters

        boardFile.push_back(tmp_fileHandle);  // Store file handle
        USBBoards.boards.push_back(boardid);  // Store board array index
        boardFPs[boardid] = tmp_fileHandle;   // Map serial (i) to fileHandle (tmp_fileHandle)
        USBBoards.activeAtStart++;            // Increment board count

        while (read(boardFile[i],buf,10)>0); //Clear buffers
        ///TODO: Needs request encoder test. Not implemented in driver yet.
        ///TODO: Clear read buffers.

        if ( write_zeros_to_board(boardid) != 0){
            ROS_ERROR("Warning: failed initial board reset (set-to-zero)");
        }
    }

    if (okboards < 2){
        ROS_ERROR("Error: failed to init two boards!  Behavior is henceforce undetermined...");
//        return 0;
    }
    //Only now we have info about number of boards and set it to number of mechanisms
    NUM_MECH = USBBoards.activeAtStart;

    return USBBoards.activeAtStart;
}

/**
 * USBShutdown() - shutsdown the USB modules. The function sets the DAC outputs to zero before shutting down.
 *
 * input: none
 *
 * \return none
 *
 */
void USBShutdown(void)
{
    uint i;

    //Reset USB driver
    for (i=0;i<boardFile.size();i++)
    {
        if (boardFile[i]) //Shutdown configured boards
        {
            if ( ioctl(boardFile[i], BRL_RESET_BOARD) != 0)
            {
                perror("ioctl error in shutdown.");
                errno = 0;
                continue; //Failed to reset board. Move to next one
            }
        }
        close(boardFile[i]);                    // Close device
        boardFile[i]=0;
    }
}

/**
Read from usb board with serial number id
* \param id serial number of board to read
* \param buffer pointer to buffer to read into
* \param len length to read
*/
int usb_read(int id, void *buffer, size_t len)
{
    int fp = boardFPs[id]; // get file pointer from serial number
    return read(fp, buffer, len);
}

/**
Write to usb board with serial number id
* \param id serial number of board to read
* \param buffer pointer to buffer to read into
* \param len length to read
*/
int usb_write(int id, void *buffer, size_t len)
{
    int fp = boardFPs[id]; // get file pointer from serial number
    return write(fp, buffer, len);       // read current enc values from board
}


/**
 * Board Reset() - reset the encoder chips on the board
 *
 * input: board number to reset
 *
 */
int usb_reset_encoders(int boardid)
{
    log_msg("Resetting encoders on board %d", boardid);

    int fp = boardFPs[boardid]; // get file pointer from serial number
    //const size_t USB_MAX_OUT_LEN = 512;
    const size_t bufsize = OUT_LENGTH;
    const char reset_byte = 0x07;
    char buf[bufsize];

    memset(buf, reset_byte, bufsize);

    write(fp,buf,bufsize); //Clear buffers
    read(fp,buf,bufsize); //Clear buffers
    return 0;
}
