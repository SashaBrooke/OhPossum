# "OhPossum" Systems Engineering 
# Master Document 
## Executive Summary 
This document consolidates all systems engineering work pertaining to project "OhPossum", a pan-tilt turret.
The engineering is staged: single axis (pan-only) prototype is to be built and tested first to de-risk interfaces and build team experience and knowledge. 
The second axis (tilt) will then take the learnings from the first stage, and apply them in order to complete the dual axis turret. 
## Acronyms
| Acronym | Text |
| --- | --- |
|ASS|Assumption|
|CT|Constraint|
|DE|Dependencies|
|FR|Functional Requirement|
| ICD | Interface Control Document |
| ID | Identification |
|ItR|Interface Requirement|
|PrR|Performance Requirement|
|PSU | Power Supply Unit|
|RL|Reliability Requirement|
| SDD | System Design Document |
|SR|Safety Requirement|
| SyRS | System Requirement Specification |
| UC | Use Case |
| UI | User Interface |
## Concept of Operations (ConOps)
### Use Cases
|Use Case ID| Use Case | Summary | 
| --- | ---| --- |
| UC-1 | Single axis bench test | Prove the mechanical, electrical and software interfaces|
| UC-2 | Pan/tilt axis bench test | Prove mechanical, electrical and software interfaces for both axis prior to full implementation
| UC-3| Manual operation | Operator uses manual inputs to control pan/tilt axis from a User Interface (UI)|
| UC-4 | Deterrent | Automatically track and identify, using a water stream to deter the target
### Operating Environments
|Environment| Conditions | Notes | 
| --- | ---| --- |
|Bench test/development| 5-40 degrees celsius, located on a desk, powered by a bench PSU | For single and dual axis bench test (UC-1 & UC-2)
|Outside - under cover | 5-40 degrees celsius, outdoor humidity, no rain on electronics |  
### Operational Modes
|Mode|Description|Entry/Exit|
|---|---|---|
|OFF|Unpowered|Power turned off/power turned on|
|STANDBY|Powered, homing functions have been performed, deterrent system disabled| Enters mode directly after power on / exits upon selection of any other state 
|MANUAL|Operator drives pan/tilt and enables/disables deterrent | User selects manual mode / exits upon selection of any other state
|AUTO-TRACK|Detects and follows a target, deterrent disabled | User selects AUTO-TRACK mode / exits upon selection of another state 
|AUTO-TRACK DETERRENT ENABLED|Detects and follows a target, when a valid target is locked, deterrent is enabled|User selects AUTO-TRACK DETERRENT ENABLED mode / exits upon selection of any other state
|EMERGENCY STOP|Emergency stop breaks motor & deterrent circuits, stopping motor motion and disabling deterrent |User enables emergency stop/user disables emergency stop and begins TBD restart process
### Support & Maintenance Needs
- Consumables:
    - Refill water reservoir 
    - Battery charging (TBD - not included in UC-1 or UC-2 but may be introduced in UC-3/4)
- Diagnostics:
    - Session log - TBD
    - Status lights - TBD
- Wear items:
    - 3D printed items can be easily printed, or upgraded to machined metal parts at a later date
    - Motors - the loads and speeds are well within safe limits - therefore wear should be slowed
- Design modularity:
    - Ability to switch deterrent - TBD
    - Ability to add another axis. Or ability to move in translation - TBD
## System Requirements Specification (SyRS)
**Note**
- T = Test
- D = Demonstration 
- I = Inspection 
- A = Analysis
### Functional Requirements
What the system must do.
|ID|Requirement|Verification Method|
|---|---|---|
|FR-01|The turret shall detect a target of size >0.1m^2 but <1.0m^2 within range specified in PR-01|Test
|FR-02| The turret camera shall acquire a video stream of the forward | Demonstrate
|FR-03|The turret shall move in pan and tilt axis |
|FR-04||
|FR-05||
|FR-06||
|FR-07||
|FR-08||
|FR-09||
|FR-10||

### Non-Functional Requirements
Performance, reliability, safety. 
|ID|Requirement|Verification Method|
|---|---|---|
|PrR-01|The turret shall perform all 
|PrR-02|
|PrR-03|The turret shall rotate about the pan axis at a speed of TBD rad/s
|PrR-04|The turret shall rotate about the tilt axis at a speed of TBD rad/s
|PrR-05|
|PrR-06|

|ID|Requirement|Verification Method|
|---|---|---|
|RL-01|
|RL-01|
|RL-01|
|RL-01|
|RL-01|

|ID|Requirement|Verification Method|
|---|---|---|
|SR-01| 
|SR-01| 
|SR-01| 
|SR-01| 

### Interfaces 
External systems, users, hardware 
|ID|Requirement|Verification Method|
|---|---|---|
|ItR-01|
|ItR-01|
|ItR-01|
|ItR-01|

### Constraints
Regulatory, environmental 
|ID|Requirement|Verification Method|
|---|---|---|
|CT-01|

### Assumptions & Dependencies
|ID|Requirement|Verification Method|
|---|---|---|
|ASS-01|

|ID|Requirement|Verification Method|
|---|---|---|
|DE-01|

## System Architecture
### State Diagram
```mermaid
stateDiagram
  [*] --> OFF
  OFF --> STANDBY: Power on (auto boot)
  STANDBY --> MANUAL: select manual mode
  STANDBY --> AUTO_TRACK: select auto mode
  STANDBY --> AUTO_TRACK_DETERRENT_ENABLED: select auto deterrent enabled mode
   AUTO_TRACK_DETERRENT_ENABLED --> Track_Active: valid target acquired
   Track_Active --> Deterrent_Active: 
  ANY_MODE -->  EMERGENCY_STOP
  OFF --> [*]
```
### Functional Decomposition
What functions the system performs.
```mermaid
flowchart TD

```
### System Breakdown
Mechanical/electrical/software.

### Physical Block Diagram

## Interface Control Document (ICD)
### Mechanical Interfaces
Mounts etc. 
### Electrical Interfaces
Power rails, signal lines.
### Data/Comms Interfaces
Protocols.
### Human/Machine Interfaces
Controls/e-stop etc. 

## System Design Document (SDD)
### Pan-Axis Design 
#### Design Trade-Offs
### Tilt-Axis Design 
#### Design Trade-Offs

## Hardware Component Selection

## Verification & Validation Plan
### Test Cases & Scenarios

## Risk Register
### Risks/Likelihood/Impact/Mitigation