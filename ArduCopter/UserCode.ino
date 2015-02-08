/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
  // put your initialisation code here
  // this will be called once at start-up
  DataCounter = 0;
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
  // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
  // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
static int DataCounter;
void userhook_MediumLoop()
{
  // put your 10Hz code here
  // Read CH 7 position to detect data switch toggle
  if(g.rc_7.control_in<=1500){
    // 7 pulled low = data on
    DataCounter++;
    Log_Write_DataCounter(); 
  }

}
static void Log_Write_DataCounter()
{
  struct log_DataCount pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
time_ms           : 
    hal.scheduler->millis(),
data_count        : 
    DataCounter
  };
  DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
  // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP

void userhook_SuperSlowLoop()
{
  // put your 1Hz code here
}

#endif



