using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0x8e2af1e708af8b8d;

# MJ Comments added :
# This code defines a protocol buffer message type in the Cap'n Proto schema language, 
# which describes the structure and fields of a "CarEvent" message 
# that represents various events related to the operation of an autonomous driving system (such as OpenPilot). 
# The CarEvent struct contains several boolean fields indicating different types of events, 
# such as enabling, disablement, warnings, etc., 
# along with an enumeration named EventName, which lists all possible names associated with these events.
# The purpose of this file seems to be defining a standard format for logging and communicating information about specific events 
# occurring within the self-driving car system. 
# By using a structured format like this, it becomes easier to parse, analyze, and react to these events programmatically.
# 이 코드는 Cap'n Proto의 스키마 언어를 사용하여 'CarEvent'라는 프로토콜 버퍼 메시지 타입을 정의합니다. 
# 자율주행 시스템(예: OpenPilot)에서 발생하는 다양한 이벤트를 나타내는 'CarEvent' 메시지의 구조와 필드를 설명합니다. 
# CarEvent 구조체에는 특정 유형의 이벤트인 
# 사용, 비활성화, 경고 등을 나타내는 몇 가지 bool 필드, 그리고 모든 가능한 이름과 연결된 EventName이라는 열거형이 포함되어 있습니다.
# 이 파일의 목적은 보다 구조화된 형식으로 자율주행차 시스템 내부에서 발생하는 특정 이벤트에 대한 정보를 기록하고 전달하는 것입니다. 
# 이러한 구조화된 형식을 사용함으로써 프로그래밍적으로 해당 이벤트를 구문 분석, 분석, 반응하기가 더 쉬워집니다.

# ******* events causing controls state machine transition *******

struct CarEvent @0x9b1657f34caf3ad3 {
  name @0 :EventName;

  # event types
  enable @1 :Bool;
  noEntry @2 :Bool;
  warning @3 :Bool;   # alerts presented only when  enabled or soft disabling
  userDisable @4 :Bool;
  softDisable @5 :Bool;
  immediateDisable @6 :Bool;
  preEnable @7 :Bool;
  permanent @8 :Bool; # alerts presented regardless of openpilot state
  overrideLateral @10 :Bool;
  overrideLongitudinal @9 :Bool;

  enum EventName @0xbaa8c5d505f727de {
    canError @0;
    steerUnavailable @1;
    wrongGear @4;
    doorOpen @5;
    seatbeltNotLatched @6;
    espDisabled @7;
    wrongCarMode @8;
    steerTempUnavailable @9;
    reverseGear @10;
    buttonCancel @11;
    buttonEnable @12;
    pedalPressed @13;  # exits active state
    preEnableStandstill @73;  # added during pre-enable state with brake
    gasPressedOverride @108;  # added when user is pressing gas with no disengage on gas
    steerOverride @114;
    cruiseDisabled @14;
    speedTooLow @17;
    outOfSpace @18;
    overheat @19;
    calibrationIncomplete @20;
    calibrationInvalid @21;
    calibrationRecalibrating @117;
    controlsMismatch @22;
    pcmEnable @23;
    pcmDisable @24;
    radarFault @26;
    brakeHold @28;
    parkBrake @29;
    manualRestart @30;
    lowSpeedLockout @31;
    plannerError @32;
    joystickDebug @34;
    steerTempUnavailableSilent @35;
    resumeRequired @36;
    preDriverDistracted @37;
    promptDriverDistracted @38;
    driverDistracted @39;
    preDriverUnresponsive @43;
    promptDriverUnresponsive @44;
    driverUnresponsive @45;
    belowSteerSpeed @46;
    lowBattery @48;
    accFaulted @51;
    sensorDataInvalid @52;
    commIssue @53;
    commIssueAvgFreq @109;
    tooDistracted @54;
    posenetInvalid @55;
    soundsUnavailable @56;
    preLaneChangeLeft @57;
    preLaneChangeRight @58;
    laneChange @59;
    lowMemory @63;
    stockAeb @64;
    ldw @65;
    carUnrecognized @66;
    invalidLkasSetting @69;
    speedTooHigh @70;
    laneChangeBlocked @71;
    relayMalfunction @72;
    stockFcw @74;
    startup @75;
    startupNoCar @76;
    startupNoControl @77;
    startupMaster @78;
    startupNoFw @104;
    fcw @79;
    steerSaturated @80;
    belowEngageSpeed @84;
    noGps @85;
    wrongCruiseMode @87;
    modeldLagging @89;
    deviceFalling @90;
    fanMalfunction @91;
    cameraMalfunction @92;
    cameraFrameRate @110;
    gpsMalfunction @94;
    processNotRunning @95;
    dashcamMode @96;
    controlsInitializing @98;
    usbError @99;
    roadCameraError @100;
    driverCameraError @101;
    wideRoadCameraError @102;
    highCpuUsage @105;
    cruiseMismatch @106;
    lkasDisabled @107;
    canBusMissing @111;
    controlsdLagging @112;
    resumeBlocked @113;
    steerTimeLimit @115;
    vehicleSensorsInvalid @116;
    locationdTemporaryError @103;
    locationdPermanentError @118;
    paramsdTemporaryError @50;
    paramsdPermanentError @119;

    radarCanErrorDEPRECATED @15;
    communityFeatureDisallowedDEPRECATED @62;
    radarCommIssueDEPRECATED @67;
    driverMonitorLowAccDEPRECATED @68;
    gasUnavailableDEPRECATED @3;
    dataNeededDEPRECATED @16;
    modelCommIssueDEPRECATED @27;
    ipasOverrideDEPRECATED @33;
    geofenceDEPRECATED @40;
    driverMonitorOnDEPRECATED @41;
    driverMonitorOffDEPRECATED @42;
    calibrationProgressDEPRECATED @47;
    invalidGiraffeHondaDEPRECATED @49;
    invalidGiraffeToyotaDEPRECATED @60;
    internetConnectivityNeededDEPRECATED @61;
    whitePandaUnsupportedDEPRECATED @81;
    commIssueWarningDEPRECATED @83;
    focusRecoverActiveDEPRECATED @86;
    neosUpdateRequiredDEPRECATED @88;
    modelLagWarningDEPRECATED @93;
    startupOneplusDEPRECATED @82;
    startupFuzzyFingerprintDEPRECATED @97;
    noTargetDEPRECATED @25;
    brakeUnavailableDEPRECATED @2;

    laneChangeManual @120;	#  A manual lane change initiated by the human driver while OpenPilot is engaged.
    emgButtonManual @121;	# An emergency stop requested manually by the driver through a designated button.
    driverSteering @122;	#  Human driver takes direct control of the vehicle's steering wheel while OpenPilot is engaged
    modeChangeOpenpilot @123;	# Switching from another assisted driving mode to OpenPilot mode.
    modeChangeDistcurv @124;	# Changing between distance and curvature based adaptive cruise control modes
    modeChangeDistance @125;       # Changing the set distance in front of the host vehicle in adaptive cruise control.
    modeChangeCurv @126;           # Adjusting the curvature setting in the adaptive cruise control system.
    modeChangeOneway @127;         # Enabling one-way traffic detection mode.
    modeChangeMaponly @128;        # Using map data exclusively for navigation without relying on sensors.
    needBrake @129;                # Indicates that the autonomous driving system requires the driver to apply brakes.
    standStill @130;               # Vehicle comes to a complete stop due to some reason.
    e2eLongAlert @131;             # End-to-end long alert notification generated by the system.
    isgActive @132;                # Intelligent Speed Assistance (ISA) feature activated.
    camSpeedDown @133;             # Camera frame rate reduced temporarily due to performance reasons.
    gapAdjusting @134;             # Automatic adjustment of following gap between vehicles in adaptive cruise control.
    resCruise @135;                # Resuming adaptive cruise control after being disabled momentarily.
    curvSpeedDown @136;            # Temporarily reducing speed around sharp curves.
    standstillResButton @137;      # Pressing the resume button at a full stop.
    routineDriveOn @138;           # Routine drive mode turned on.
    lkasEnabled @139;              # Lane Keeping Assist System (LKAS) has been enabled.
    cutinDetection @140;           # Detecting other vehicles cutting into the path of the host vehicle.
    gearNotD @141;                 # Gear not in Drive ('D') position.
    unSleepMode @142;              # Exiting sleep mode in the autonomous driving system.
    speedBump @143;                # Detection of a speed bump.
    sccDriverOverride @144;        # Driver taking temporary control of acceleration and deceleration in SCC (Smart Cruise Control) mode.
    doNotDisturb @145;             # Do Not Disturb mode activated.
    chimeAtResume @146;            # Sound played upon resumption of automated driving.
    autoHold @147;                 # Auto Hold feature activated, allowing automatic engagement and release of parking brake under certain conditions.

	# laneChangeManual - OpenPilot이 참여 중일 때 사람 운전자가 수동으로 레인 변경을 시도합니다.
	# emgButtonManual - 사람 운전자가 지정된 단추를 통해 긴급 정지를 요청합니다.
	# driverSteering - OpenPilot이 참여 중일 때 사람 운전자가 직접 핸들을 조작합니다.
	# modeChangeOpenpilot - 다른 지원 주행 모드에서 OpenPilot 모드로 전환합니다.
	# modeChangeDistcurv - 거리 기준과 커브 기준의 적응 크루즈 컨트롤 모드 간의 전환.
	# modeChangeDistance - 적응 크루즈 컨트롤에서 호스트 차량 앞의 설정 거리 조정.
	# modeChangeCurv - 적응 크루즈 컨트롤 시스템의 커브 설정 조정.
	# modeChangeOneway - 일방통행 감지 모드 활성화.
	# modeChangeMaponly - 센서를 사용하지 않고 오직 지도 데이터만 사용하여 진행합니다.
	# needBrake - 자율주행 시스템이 사람 운전자에게 브레이크를 적용하도록 요구합니다.
	# standStill - 특정 이유로 차량이 완전히 정지합니다.
	# e2eLongAlert - 끝까지 긴 알림 생성된 시스템.
	# isgActive - 지능형 속도 제한 기능(Intelligent Speed Assistance, ISA)이 활성화됩니다.
	# camSpeedDown - 성능 이유로 일시적으로 카메라 프레임 레이트를 줄입니다.
	# gapAdjusting - 적응 크루즈 컨트롤에서 차량 사이의 후속 간격 자동 조정.
	# resCruise - 일시적으로 비활성화된 적응 크루즈 컨트롤을 재개합니다.
	# curvSpeedDown - 굴곡이 심한 경로 주변에서 일시적으로 속도를 줄입니다.
	# standstillResButton - 정지 상태에서 복원 단추를 누릅니다.
	# routineDriveOn - 일상 주행 모드가 켜짐.
	# lkasEnabled - 라인 유지 보조 시스템(Lane Keeping Assist System, LKAS)이 활성화됨.
	# cutinDetection - 호스트 차량 경로에 차량이 침입하는 것을 감지합니다.
	# gearNotD - 기어가 'D'(드라이브) 위치가 아닙니다.
	# unSleepMode - 자율주행 시스템의 수면 모드를 종료합니다.
	# speedBump - Geschwindigkeitsstoß absorbierende Element erkannt.
	# sccDriverOverride - SCC(스마트 크루즈 컨트롤) 모드에서 운전자가 일시적으로 가속 및 감속을 제어합니다.
	# doNotDisturb - 방해금지 모드가 활성화됩니다.
	# chimeAtResume - 자동 주행 재개 시 소음 출력.
	# autoHold - 특정 조건에서 주차 브레이크를 자동으로 참여 및 해제하는 자동 보강 기능이 활성화됩니다.
  }
}

# MJ Comments added :
# This text describes a data structure called CarState which contains information about different aspects of a vehicle 
# being controlled by an automated driving system like OpenPilot.
# It includes details about the car's current state at 100 Hz frequency, 
# including its speed, acceleration, pedals position, steering angle, cruising status, etc.
# Regarding '@48', it refers to the field name 'tpms'. 
# In this context, '@48' indicates the bit alignment offset within the serialized binary format 
# representing the entire CarState structure. 
# When encoding or decoding the whole structure into bytes, 
# each element will start at their specified offset positions. 
# Here, 'tpms' starts at the 48th bit.
# In simpler terms, you don't have to worry much about those numbers ('@48') 
# unless you work on serialization, deserialization, or memory management tasks involving direct manipulation of bits and byte streams. 
# They mainly help ensure proper placement and organization of elements 
# inside the final serialized representation of the complex data structures like CarState.

# 이 텍스트는 자율주행 시스템으로부터 차량의 현재 상태를 설명하는 CarState라는 데이터 구조입니다. 
# 이 정보에는 속도, 가속도, 페달 위치, 방향 전환 각도, 크루즈 상태와 같은 차량의 여러 측면이 포함되어 있습니다. 
# 이 모든 것은 자동 주행 시스템과 같은 OpenPilot과 같이 차량을 제어하는 시스템에서 사용됩니다.
# '@48'은 'tpms'라는 필드 이름을 가리키고 있습니다. 
# 여기서 '@48'은 일련의 바이너리 형식으로 직렬화된 CarState 구조체 내에서 해당 필드의 비트 Alignment 오프셋을 나타냅니다. 
# 전체 CarState 구조체를 바이트 스트림으로 인코딩하거나 디코딩할 때, 각 원소는 지정된 오프셋 위치에서 시작합니다. 
# 여기서 'tpms'는 48번째 비트에서 시작합니다.
# 더 간단히 말해, '@48'와 같은 숫자들에 대해서는 직접 바이트 및 비트 수준의 직렬화, 역직렬화, 또는 메모리 관리 작업을 담당하지 않는 한 걱정할 필요가 없습니다. 
# 그들은 복잡한 데이터 구조체(CarState)의 최종 직렬화된 표현에서 각 요소의 올바른 배치와 조직을 보장하는 데 도움이 되는 것일 뿐입니다.

# ******* main car state @ 100hz *******
# all speeds in m/s

struct CarState {
  events @13 :List(CarEvent);

  # CAN health
  canValid @26 :Bool;       # invalid counter/checksums
  canTimeout @40 :Bool;     # CAN bus dropped out

  # car speed
  vEgo @1 :Float32;          # best estimate of speed
  aEgo @16 :Float32;         # best estimate of acceleration
  vEgoRaw @17 :Float32;      # unfiltered speed from CAN sensors
  vEgoCluster @44 :Float32;  # best estimate of speed shown on car's instrument cluster, used for UI

  yawRate @22 :Float32;     # best estimate of yaw rate
  standstill @18 :Bool;
  wheelSpeeds @2 :WheelSpeeds;

  # gas pedal, 0.0-1.0
  gas @3 :Float32;        # this is user pedal only
  gasPressed @4 :Bool;    # this is user pedal only

  engineRpm @46 :Float32;

  # brake pedal, 0.0-1.0
  brake @5 :Float32;      # this is user pedal only
  brakePressed @6 :Bool;  # this is user pedal only
  regenBraking @45 :Bool; # this is user pedal only
  parkingBrake @39 :Bool;
  brakeHoldActive @38 :Bool;

  # steering wheel
  steeringAngleDeg @7 :Float32;
  steeringAngleOffsetDeg @37 :Float32; # Offset betweens sensors in case there multiple
  steeringRateDeg @15 :Float32;
  steeringTorque @8 :Float32;      # TODO: standardize units
  steeringTorqueEps @27 :Float32;  # TODO: standardize units
  steeringPressed @9 :Bool;        # if the user is using the steering wheel
  steerFaultTemporary @35 :Bool;   # temporary EPS fault
  steerFaultPermanent @36 :Bool;   # permanent EPS fault
  stockAeb @30 :Bool;
  stockFcw @31 :Bool;
  espDisabled @32 :Bool;
  accFaulted @42 :Bool;
  carFaultedNonCritical @47 :Bool;  # some ECU is faulted, but car remains controllable

  # cruise state
  cruiseState @10 :CruiseState;

  # gear
  gearShifter @14 :GearShifter;

  # button presses
  buttonEvents @11 :List(ButtonEvent);
  leftBlinker @20 :Bool;
  rightBlinker @21 :Bool;
  genericToggle @23 :Bool;

  # lock info
  doorOpen @24 :Bool;
  seatbeltUnlatched @25 :Bool;

  # clutch (manual transmission only)
  clutchPressed @28 :Bool;

  # blindspot sensors
  leftBlindspot @33 :Bool; # Is there something blocking the left lane change
  rightBlindspot @34 :Bool; # Is there something blocking the right lane change

  fuelGauge @41 :Float32; # battery or fuel tank level from 0.0 to 1.0
  charging @43 :Bool;

  # opkr-tpms
  tpms @48 :TPMS;               # Tire Pressure Monitor System information
  
  radarDistance @49 :Float32;   # Distance from car to detected obstacle using radar
  standStill @50 :Bool;         # Indicates whether the vehicle is standing still
  vSetDis @51 :Float32;          # Set driving speed when activating cruise control
  cruiseButtons @52 :Float32;   # Cruise control button status
  cruiseAccStatus @53 :Bool;     # Acceleration pedal position during cruise control
  driverAcc @54 :Bool;           # Driver accelerator pedal position
  autoHold @55 :Bool;            # Auto Hold feature flag
  cruiseGapSet @56 :UInt8;       # Configured gap between cars in front during cruise control
  
  # opkr
  safetyDist @57 :Float32;      # Safety distance threshold
  safetySign @58 :Float32;      # Threshold value for recognizing road signs
  vEgoOP @59 :Float32;          # Vehicle speed calculated by OpenPilot
  gearStep @60 :Int8;           # Current transmission gear step
  isMph @61 :Bool;              # Flag indicating whether speed units are miles per hour (mph) or kilometers per hour (km/h)
  aReqValue @62 :Float32;        # Requested acceleration value
  chargeMeter @63 :Float32;     # Battery level indicator
  brakeLights @64 :Bool;         # Brake lights activation flag
  
  struct TPMS {             # Structure containing tire pressure data
    unit @0 :Int8;          # Unit type (psi, kpa, bar, etc.)
    fl @1 :Float32;         # Front left tire pressure
    fr @2 :Float32;         # Front right tire pressure
    rl @3 :Float32;         # Rear left tire pressure
    rr @4 :Float32;         # Rear right tire pressure
  }

  struct WheelSpeeds {
    # optional wheel speeds
    fl @0 :Float32;
    fr @1 :Float32;
    rl @2 :Float32;
    rr @3 :Float32;
  }

  struct CruiseState {
    enabled @0 :Bool;
    speed @1 :Float32;
    speedCluster @6 :Float32;  # Set speed as shown on instrument cluster
    available @2 :Bool;
    speedOffset @3 :Float32;
    standstill @4 :Bool;
    nonAdaptive @5 :Bool;

    # atom
    modeSel @7 :Int16;
    cruiseSwState @8 :Int16;
    accActive @9 :Bool;
    gapSet @10 :Int16;
  }

  enum GearShifter {
    unknown @0;
    park @1;
    drive @2;
    neutral @3;
    reverse @4;
    sport @5;
    low @6;
    brake @7;
    eco @8;
    manumatic @9;
  }

  # send on change
  struct ButtonEvent {
    pressed @0 :Bool;
    type @1 :Type;

    enum Type {
      unknown @0;
      leftBlinker @1;
      rightBlinker @2;
      accelCruise @3;
      decelCruise @4;
      cancel @5;
      altButton1 @6;
      altButton2 @7;
      altButton3 @8;
      setCruise @9;
      resumeCruise @10;
      gapAdjustCruise @11;
    }
  }

  # deprecated
  errorsDEPRECATED @0 :List(CarEvent.EventName);
  brakeLightsDEPRECATED @19 :Bool;
  steeringRateLimitedDEPRECATED @29 :Bool;
  canMonoTimesDEPRECATED @12: List(UInt64);
}

# ******* radar state @ 20hz *******

struct RadarData @0x888ad6581cf0aacb {
  errors @0 :List(Error);
  points @1 :List(RadarPoint);

  enum Error {
    canError @0;
    fault @1;
    wrongConfig @2;
  }

  # similar to LiveTracks
  # is one timestamp valid for all? I think so
  struct RadarPoint {
    trackId @0 :UInt64;  # no trackId reuse

    # these 3 are the minimum required
    dRel @1 :Float32; # m from the front bumper of the car
    yRel @2 :Float32; # m
    vRel @3 :Float32; # m/s

    # these are optional and valid if they are not NaN
    aRel @4 :Float32; # m/s^2
    yvRel @5 :Float32; # m/s

    # some radars flag measurements VS estimates
    measured @6 :Bool;
  }

  # deprecated
  canMonoTimesDEPRECATED @2 :List(UInt64);
}

# ******* car controls @ 100hz *******

struct CarControl {
  # must be true for any actuator commands to work
  enabled @0 :Bool;
  latActive @11: Bool;
  longActive @12: Bool;

  # Actuator commands as computed by controlsd
  actuators @6 :Actuators;

  leftBlinker @15: Bool;
  rightBlinker @16: Bool;

  # Any car specific rate limits or quirks applied by
  # the CarController are reflected in actuatorsOutput
  # and matches what is sent to the car
  actuatorsOutput @10 :Actuators;

  orientationNED @13 :List(Float32);
  angularVelocity @14 :List(Float32);

  cruiseControl @4 :CruiseControl;
  hudControl @5 :HUDControl;

  struct Actuators {
    # range from 0.0 - 1.0
    gas @0: Float32;
    brake @1: Float32;
    # range from -1.0 - 1.0
    steer @2: Float32;
    # value sent over can to the car
    steerOutputCan @8: Float32;
    steeringAngleDeg @3: Float32;

    curvature @7: Float32;

    speed @6: Float32; # m/s
    accel @4: Float32; # m/s^2
    longControlState @5: LongControlState;

    # opkr
    oaccel @9: Float32; # m/s^2
    
    enum LongControlState @0xe40f3a917d908282{
      off @0;
      pid @1;
      stopping @2;
      starting @3;
    }
  }

  struct CruiseControl {
    cancel @0: Bool;
    resume @1: Bool;
    override @4: Bool;
    speedOverrideDEPRECATED @2: Float32;
    accelOverrideDEPRECATED @3: Float32;
  }

  struct HUDControl {
    speedVisible @0: Bool;
    setSpeed @1: Float32;
    lanesVisible @2: Bool;
    leadVisible @3: Bool;
    visualAlert @4: VisualAlert;
    audibleAlert @5: AudibleAlert;
    rightLaneVisible @6: Bool;
    leftLaneVisible @7: Bool;
    rightLaneDepart @8: Bool;
    leftLaneDepart @9: Bool;
    vFuture @10:Float32;
    vFutureA @11:Float32;

    enum VisualAlert {
      # these are the choices from the Honda
      # map as good as you can for your car
      none @0;
      fcw @1;
      steerRequired @2;
      brakePressed @3;
      wrongGear @4;
      seatbeltUnbuckled @5;
      speedTooHigh @6;
      ldw @7;
    }

    enum AudibleAlert {
      none @0;

      engage @1;
      disengage @2;
      refuse @3;

      warningSoft @4;
      warningImmediate @5;

      prompt @6;
      promptRepeat @7;
      promptDistracted @8;
      warning @9;
      dingdong @10;
    }
  }

  gasDEPRECATED @1 :Float32;
  brakeDEPRECATED @2 :Float32;
  steeringTorqueDEPRECATED @3 :Float32;
  activeDEPRECATED @7 :Bool;
  rollDEPRECATED @8 :Float32;
  pitchDEPRECATED @9 :Float32;
}

# ****** car param ******

struct CarParams {
  carName @0 :Text;
  carFingerprint @1 :Text;
  fuzzyFingerprint @55 :Bool;

  notCar @66 :Bool;  # flag for non-car robotics platforms

  enableGasInterceptor @2 :Bool;
  pcmCruise @3 :Bool;        # is openpilot's state tied to the PCM's cruise state?
  enableDsu @5 :Bool;        # driving support unit
  enableBsm @56 :Bool;       # blind spot monitoring
  flags @64 :UInt32;         # flags for car specific quirks
  experimentalLongitudinalAvailable @71 :Bool;
  experimentalLong @96 :Bool;
  experimentalLongAlt @97 :Bool;

  minEnableSpeed @7 :Float32;
  minSteerSpeed @8 :Float32;
  smoothSteer @95 :SmoothSteerData;
  safetyConfigs @62 :List(SafetyConfig);
  alternativeExperience @65 :Int16;      # panda flag for features like no disengage on gas

  # Car docs fields
  maxLateralAccel @68 :Float32;
  autoResumeSng @69 :Bool;               # describes whether car can resume from a stop automatically

  # things about the car in the manual
  mass @17 :Float32;            # [kg] curb weight: all fluids no cargo
  wheelbase @18 :Float32;       # [m] distance from rear axle to front axle
  centerToFront @19 :Float32;   # [m] distance from center of mass to front axle
  steerRatio @20 :Float32;      # [] ratio of steering wheel angle to front wheel angle
  steerRatioRear @21 :Float32;  # [] ratio of steering wheel angle to rear wheel angle (usually 0)

  # things we can derive
  rotationalInertia @22 :Float32;    # [kg*m2] body rotational inertia
  tireStiffnessFactor @72 :Float32;  # scaling factor used in calculating tireStiffness[Front,Rear]
  tireStiffnessFront @23 :Float32;   # [N/rad] front tire coeff of stiff
  tireStiffnessRear @24 :Float32;    # [N/rad] rear tire coeff of stiff

  longitudinalTuning @25 :LongitudinalPIDTuning;
  lateralParams @48 :LateralParams;
  lateralTuning :union {
    pid @26 :LateralPIDTuning;
    indi @27 :LateralINDITuning;
    lqr @40 :LateralLQRTuning;
    torque @67 :LateralTorqueTuning;
    atom @94 :LateralATOMTuning;
  }

  steerLimitAlert @28 :Bool;
  steerLimitTimer @47 :Float32;  # time before steerLimitAlert is issued

  vEgoStopping @29 :Float32; # Speed at which the car goes into stopping state
  vEgoStarting @59 :Float32; # Speed at which the car goes into starting state
  stoppingControl @31 :Bool; # Does the car allow full control even at lows speeds when stopping
  steerControlType @34 :SteerControlType;
  radarUnavailable @35 :Bool; # True when radar objects aren't visible on CAN or aren't parsed out
  stopAccel @60 :Float32; # Required acceleration to keep vehicle stationary
  stoppingDecelRate @52 :Float32; # m/s^2/s while trying to stop
  startAccel @32 :Float32; # Required acceleration to get car moving
  startingState @70 :Bool; # Does this car make use of special starting state

  steerActuatorDelay @36 :Float32; # Steering wheel actuator delay in seconds
  longitudinalActuatorDelayLowerBound @61 :Float32; # Gas/Brake actuator delay in seconds, lower bound
  longitudinalActuatorDelayUpperBound @58 :Float32; # Gas/Brake actuator delay in seconds, upper bound
  openpilotLongitudinalControl @37 :Bool; # is openpilot doing the longitudinal control?
  carVin @38 :Text; # VIN number queried during fingerprinting
  dashcamOnly @41: Bool;
  transmissionType @43 :TransmissionType;
  carFw @44 :List(CarFw);

  radarTimeStep @45: Float32 = 0.05;  # time delta between radar updates, 20Hz is very standard
  fingerprintSource @49: FingerprintSource;
  networkLocation @50 :NetworkLocation;  # Where Panda/C2 is integrated into the car's CAN network

  wheelSpeedFactor @63 :Float32; # Multiplier on wheels speeds to computer actual speeds

  struct SafetyConfig {
    safetyModel @0 :SafetyModel;
    safetyParam @3 :UInt16;
    safetyParamDEPRECATED @1 :Int16;
    safetyParam2DEPRECATED @2 :UInt32;
  }

  # opkr
  mdpsBus @73: Int8;
  sasBus @74: Int8;
  sccBus @75: Int8;
  fcaBus @76: Int8;
  bsmAvailable @77: Bool;
  lfaAvailable @78: Bool;
  lvrAvailable @79: Bool;
  evgearAvailable @80: Bool;
  emsAvailable @81: Bool;
  standStill @82: Bool;
  vCruisekph @83: Float32;
  resSpeed @84: Float32;
  vFuture @85: Float32;
  aqValue @86: Float32;
  aqValueRaw @87: Float32;
  vFutureA @88: Float32;
  autoHoldAvailable @89 :Bool;
  scc13Available @90 :Bool;
  scc14Available @91 :Bool;
  lfaHdaAvailable @92 :Bool;
  navAvailable @93 :Bool;

  struct SmoothSteerData
  {
    method @0: Int8;
    maxSteeringAngle @1 :Float32;
    maxDriverAngleWait @2 :Float32;
    maxSteerAngleWait @3 :Float32;
    driverAngleWait @4 :Float32;
  }

  struct LateralParams {
    torqueBP @0 :List(Int32);
    torqueV @1 :List(Int32);
  }

  struct LateralATOMTuning {
    lqr @0 :LateralLQRTuning;
    torque @1 :LateralTorqueTuning;
    indi @2 :LateralINDITuning;
    pid @3 :LateralPIDTuning;
  }

  struct LateralPIDTuning {
    kpBP @0 :List(Float32);
    kpV @1 :List(Float32);
    kiBP @2 :List(Float32);
    kiV @3 :List(Float32);
    kf @4 :Float32;
    kd @5 :Float32;
  }

  struct LateralTorqueTuning {
    useSteeringAngle @0 :Bool;
    kp @1 :Float32;
    ki @2 :Float32;
    friction @3 :Float32;
    kf @4 :Float32;
    steeringAngleDeadzoneDeg @5 :Float32;
    latAccelFactor @6 :Float32;
    latAccelOffset @7 :Float32;
  }

  struct LongitudinalPIDTuning {
    kpBP @0 :List(Float32);
    kpV @1 :List(Float32);
    kiBP @2 :List(Float32);
    kiV @3 :List(Float32);
    kf @6 :Float32;
    deadzoneBP @4 :List(Float32);
    deadzoneV @5 :List(Float32);
    kdBP @7 :List(Float32) = [0.];
    kdV @8 :List(Float32) = [0.];
    kfBP @9 :List(Float32);
    kfV @10 :List(Float32);
  }

  struct LateralINDITuning {
    outerLoopGainBP @4 :List(Float32);
    outerLoopGainV @5 :List(Float32);
    innerLoopGainBP @6 :List(Float32);
    innerLoopGainV @7 :List(Float32);
    timeConstantBP @8 :List(Float32);
    timeConstantV @9 :List(Float32);
    actuatorEffectivenessBP @10 :List(Float32);
    actuatorEffectivenessV @11 :List(Float32);

    outerLoopGainDEPRECATED @0 :Float32;
    innerLoopGainDEPRECATED @1 :Float32;
    timeConstantDEPRECATED @2 :Float32;
    actuatorEffectivenessDEPRECATED @3 :Float32;
  }

  struct LateralLQRTuning {
    scale @0 :Float32;
    ki @1 :Float32;
    dcGain @2 :Float32;

    # State space system
    a @3 :List(Float32);
    b @4 :List(Float32);
    c @5 :List(Float32);

    k @6 :List(Float32);  # LQR gain
    l @7 :List(Float32);  # Kalman gain
  }

  enum SafetyModel {
    silent @0;
    hondaNidec @1;
    toyota @2;
    elm327 @3;
    gm @4;
    hondaBoschGiraffe @5;
    ford @6;
    cadillac @7;
    hyundai @8;
    chrysler @9;
    tesla @10;
    subaru @11;
    gmPassive @12;
    mazda @13;
    nissan @14;
    volkswagen @15;
    toyotaIpas @16;
    allOutput @17;
    gmAscm @18;
    noOutput @19;  # like silent but without silent CAN TXs
    hondaBosch @20;
    volkswagenPq @21;
    subaruPreglobal @22;  # pre-Global platform
    hyundaiLegacy @23;
    hyundaiCommunity1 @24;
    volkswagenMlb @25;
    hongqi @26;
    body @27;
    hyundaiCanfd @28;
    hyundaiCommunity2 @29;
    hyundaiCommunity1Legacy @30;
    volkswagenMqbEvo @31;
  }

  enum SteerControlType {
    torque @0;
    angle @1;
    curvature @2;
  }

  enum TransmissionType {
    unknown @0;
    automatic @1;  # Traditional auto, including DSG
    manual @2;  # True "stick shift" only
    direct @3;  # Electric vehicle or other direct drive
    cvt @4;
  }

  struct CarFw {
    ecu @0 :Ecu;
    fwVersion @1 :Data;
    address @2 :UInt32;
    subAddress @3 :UInt8;
    responseAddress @4 :UInt32;
    request @5 :List(Data);
    brand @6 :Text;
    bus @7 :UInt8;
    logging @8 :Bool;
    obdMultiplexing @9 :Bool;
  }

  enum Ecu {
    eps @0;
    abs @1;
    fwdRadar @2;
    fwdCamera @3;
    engine @4;
    unknown @5;
    transmission @8; # Transmission Control Module
    hybrid @18; # hybrid control unit, e.g. Chrysler's HCP, Honda's IMA Control Unit, Toyota's hybrid control computer
    srs @9; # airbag
    gateway @10; # can gateway
    hud @11; # heads up display
    combinationMeter @12; # instrument cluster
    electricBrakeBooster @15;
    shiftByWire @16;
    adas @19;
    cornerRadar @21;
    hvac @20;
    parkingAdas @7;  # parking assist system ECU, e.g. Toyota's IPAS, Hyundai's RSPA, etc.
    epb @22;  # electronic parking brake
    telematics @23;
    body @24;  # body control module

    # Toyota only
    dsu @6;

    # Honda only
    vsa @13; # Vehicle Stability Assist
    programmedFuelInjection @14;

    debug @17;
  }

  enum FingerprintSource {
    can @0;
    fw @1;
    fixed @2;
  }

  enum NetworkLocation {
    fwdCamera @0;  # Standard/default integration at LKAS camera
    gateway @1;    # Integration at vehicle's CAN gateway
  }

  enableCameraDEPRECATED @4 :Bool;
  enableApgsDEPRECATED @6 :Bool;
  steerRateCostDEPRECATED @33 :Float32;
  isPandaBlackDEPRECATED @39 :Bool;
  hasStockCameraDEPRECATED @57 :Bool;
  safetyParamDEPRECATED @10 :Int16;
  safetyModelDEPRECATED @9 :SafetyModel;
  safetyModelPassiveDEPRECATED @42 :SafetyModel = silent;
  minSpeedCanDEPRECATED @51 :Float32;
  communityFeatureDEPRECATED @46: Bool;
  startingAccelRateDEPRECATED @53 :Float32;
  steerMaxBPDEPRECATED @11 :List(Float32);
  steerMaxVDEPRECATED @12 :List(Float32);
  gasMaxBPDEPRECATED @13 :List(Float32);
  gasMaxVDEPRECATED @14 :List(Float32);
  brakeMaxBPDEPRECATED @15 :List(Float32);
  brakeMaxVDEPRECATED @16 :List(Float32);
  directAccelControlDEPRECATED @30 :Bool;
  maxSteeringAngleDegDEPRECATED @54 :Float32;
}
