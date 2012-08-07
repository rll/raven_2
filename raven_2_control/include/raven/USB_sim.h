/**
Functions for communicating with the raven simulator.
NOTE: This file is included within the USB_init.c to provide communication with the simulator
*/

/**
 * USBInit() - initialize the USB modules
 *
 * input: none
 *
 * \return status of initialization: 0 if no USB board found, USB_INIT_ERROR if an error was encountered or TRUE if initialized successfully
 *
 */


int USBInit()
{
  int i, configured = 0;
  char buf[10]; //buffer to be used for clearing usb read buffers

  USBBoards.activeAtStart=0;
  for(i=0;i<NUM_BOARDS;i++)
  {
      boardStatus[i]=USB_INIT_ERROR; //Reset to true if board is successfully initialized in the following section
      if(i<10)
      {
          boardStr[BOARD_FILE_STR_LEN-3]=48+i;
          boardStr[BOARD_FILE_STR_LEN-2]=0;
      }
      else
      {
          boardStr[BOARD_FILE_STR_LEN-2]=48+i;
          boardStr[BOARD_FILE_STR_LEN-1]=0;
      }
      log_msg("open\n");
      boardFile[i] = open(boardStr, O_RDWR);    // open board chardev
      if (boardFile[i] <=0 ){
        perror("Error connecting to device");
        errno=0;
        continue; //Failed to open board, move to next one
        //exit(-1);
      }

      log_msg("ioctl\n");
      if ( ioctl(boardFile[i], BRL_RESET_BOARD) != 0) {
        perror("ioctl error");
        errno = 0;
        continue; //Failed to reset board. Move to next one
        //exit(-1);
      }
      boardStatus[i]=TRUE;
      configured=TRUE; //We have at least one board configured. Return true.
      USBBoards.activeAtStart++;
      USBBoards.boards[USBBoards.activeAtStart-1]=i;
      while(read(boardFile[i],buf,10)>=0); //Clear buffers
      ///TODO: Needs request encoder test. Not implemented in driver yet.
      ///TODO: Clear read buffers.
  }
  //Only now we have info about number of boards and set it to number of mechanisms
  NUM_MECH = USBBoards.activeAtStart;


  return configured;
}
