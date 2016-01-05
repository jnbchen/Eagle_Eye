void execute () {
  try{
    // infinite loop:
    while (true) {
      // get vehicle pose from blackboard
      Pose vp = BBOARD->getVehiclePose ();
      // write vehicle pose to screen
      LOUT (vp.position)
      // wait until new pose is available
      BBOARD->waitForVehiclePose ();
      // test whether thread should be canceled
      boost::this_thread::interruption_point();
    }
  }catch(boost::thread_interrupted&){;}
}