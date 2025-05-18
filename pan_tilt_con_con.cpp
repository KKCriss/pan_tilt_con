void jsonCmdReceiveHandler() {
    int cmdType = jsonCmdReceive["T"].as<int>();
    switch (cmdType) {
    case CMD_GIMBAL_CTRL_SIMPLE:
        gimbalCtrlSimple(
            jsonCmdReceive["X"],
            jsonCmdReceive["Y"],
            jsonCmdReceive["SPD"],
            jsonCmdReceive["ACC"]); break;
    case CMD_GIMBAL_CTRL_MOVE:
        gimbalCtrlMove(
            jsonCmdReceive["X"],
            jsonCmdReceive["Y"],
            jsonCmdReceive["SX"],
            jsonCmdReceive["SY"]); break;
    case CMD_GIMBAL_CTRL_STOP:
        gimbalCtrlStop(); break;
    case CMD_HEART_BEAT_SET:
        changeHeartBeatDelay(
            jsonCmdReceive["cmd"]); break;
    case CMD_GIMBAL_STEADY:
        gimbalSteadySet(
            jsonCmdReceive["s"],
            jsonCmdReceive["y"]); break;
    }
}
