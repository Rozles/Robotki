
"use strict";

let KeyboardInput = require('./KeyboardInput.js');
let PowerSystemEvent = require('./PowerSystemEvent.js');
let ScanAngle = require('./ScanAngle.js');
let SensorState = require('./SensorState.js');
let Sound = require('./Sound.js');
let BumperEvent = require('./BumperEvent.js');
let DockInfraRed = require('./DockInfraRed.js');
let RobotStateEvent = require('./RobotStateEvent.js');
let MotorPower = require('./MotorPower.js');
let ExternalPower = require('./ExternalPower.js');
let DigitalInputEvent = require('./DigitalInputEvent.js');
let Led = require('./Led.js');
let CliffEvent = require('./CliffEvent.js');
let ControllerInfo = require('./ControllerInfo.js');
let VersionInfo = require('./VersionInfo.js');
let DigitalOutput = require('./DigitalOutput.js');
let ButtonEvent = require('./ButtonEvent.js');
let WheelDropEvent = require('./WheelDropEvent.js');
let AutoDockingFeedback = require('./AutoDockingFeedback.js');
let AutoDockingGoal = require('./AutoDockingGoal.js');
let AutoDockingResult = require('./AutoDockingResult.js');
let AutoDockingActionGoal = require('./AutoDockingActionGoal.js');
let AutoDockingAction = require('./AutoDockingAction.js');
let AutoDockingActionResult = require('./AutoDockingActionResult.js');
let AutoDockingActionFeedback = require('./AutoDockingActionFeedback.js');

module.exports = {
  KeyboardInput: KeyboardInput,
  PowerSystemEvent: PowerSystemEvent,
  ScanAngle: ScanAngle,
  SensorState: SensorState,
  Sound: Sound,
  BumperEvent: BumperEvent,
  DockInfraRed: DockInfraRed,
  RobotStateEvent: RobotStateEvent,
  MotorPower: MotorPower,
  ExternalPower: ExternalPower,
  DigitalInputEvent: DigitalInputEvent,
  Led: Led,
  CliffEvent: CliffEvent,
  ControllerInfo: ControllerInfo,
  VersionInfo: VersionInfo,
  DigitalOutput: DigitalOutput,
  ButtonEvent: ButtonEvent,
  WheelDropEvent: WheelDropEvent,
  AutoDockingFeedback: AutoDockingFeedback,
  AutoDockingGoal: AutoDockingGoal,
  AutoDockingResult: AutoDockingResult,
  AutoDockingActionGoal: AutoDockingActionGoal,
  AutoDockingAction: AutoDockingAction,
  AutoDockingActionResult: AutoDockingActionResult,
  AutoDockingActionFeedback: AutoDockingActionFeedback,
};
