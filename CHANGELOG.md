<a name="unreleased"></a>
## [Unreleased]

### Fix
- **security-context:** :bug: prevent nil pointer ref in container security context


<a name="v0.2.5-alpha.17"></a>
## [v0.2.5-alpha.17] - 2023-06-21

<a name="v0.2.5-alpha.16"></a>
## [v0.2.5-alpha.16] - 2023-06-21

<a name="v0.2.5-alpha.15"></a>
## [v0.2.5-alpha.15] - 2023-06-06
### Fix
- **ingress:** :bug: fix ingress annotations


<a name="v0.2.5-alpha.13"></a>
## [v0.2.5-alpha.13] - 2023-05-29
### Feat
- **image:** :rocket: select robot image by querying versioning map

### Fix
- **launchmanager:** :bug: fix checking irrelevant type equalty in webhooks
- **vdi:** :bug: fix vdi ingress and connection url


<a name="v0.2.5-alpha.12"></a>
## [v0.2.5-alpha.12] - 2023-05-25

<a name="v0.2.5-alpha.11"></a>
## [v0.2.5-alpha.11] - 2023-05-25
### Fix
- **typo:** fix typo in logs


<a name="v0.2.5-alpha.10"></a>
## [v0.2.5-alpha.10] - 2023-05-24
### Fix
- **ide:** update ide host and path values in ingress
- **ingress:** :bug: update ingress configurations
- **oauth2:** fix oauth2 url
- **vdi:** update vdi host and path values in ingress


<a name="v0.2.5-alpha.9"></a>
## [v0.2.5-alpha.9] - 2023-05-08
### Fix
- **scheduling:** stop assigning status to all steps in deletion attempts
- **steps:** fix instance scheduling of buildmanager steps


<a name="v0.2.5-alpha.8"></a>
## [v0.2.5-alpha.8] - 2023-05-03
### Feat
- **launch:** :rocket: add custom launch option

### Fix
- **build-manager:** :bug: fix cluster selection for steps
- **build-manager:** make phase ready if no step is assigned in instance
- **fastdds:** :bug: fix fastdds config, use only udpv4
- **import:** fix broken imports


<a name="v0.2.5-alpha.6"></a>
## [v0.2.5-alpha.6] - 2023-04-18

<a name="v0.2.5-alpha.5"></a>
## [v0.2.5-alpha.5] - 2023-04-18

<a name="v0.2.5-alpha.4"></a>
## [v0.2.5-alpha.4] - 2023-04-13

<a name="v0.2.5-alpha.3"></a>
## [v0.2.5-alpha.3] - 2023-04-11
### Fix
- **field:** change problematic dir names because of fabric8 code generation issues


<a name="v0.2.5-alpha.2"></a>
## [v0.2.5-alpha.2] - 2023-04-07
### Feat
- **gpu:** :rocket: watch volatile gpu utilization of node
- **metrics-exporter:** follow gpu and network metrics from host
- **network:** :rocket: watch network load of node network interfaces
- **vdi:** select vdi resolution


<a name="v0.2.5-alpha.1"></a>
## v0.2.5-alpha.1 - 2023-03-01
### Feat
- **client:** create client for robot
- **discovery-server:** :rocket: manage a discovery server attached to the robot
- **dynamic-ws:** make workspaces changeable dynamically
- **metrics:** enable multiple ros 2 distribution option
- **metrics:** enable observing robot's metrics
- **rmw-implementation:** enable selecting rmw implementation
- **robot:** :rocket: manage main robot assets
- **robot-build:** add automation to the building process of robot
- **robot-dev-suite:** manage development suites attached to a robot
- **robot-ide:** provision cloud ide attached to robot
- **robot-launch:** add automation to the launching process of robot
- **robot-vdi:** provision virtual desktop attached to robot
- **ros2-run:** :rocket: support ros2 run command
- **rosbridge:** :rocket: manage ros bridge server for robot instance
- **volume:** :rocket: provision robot's volumes dynamically
- **volumes:** :rocket: configure robot's volumes
- **workspaces:** :rocket: prepare workspaces according to the definitions in robot manifest

### Fix
- **api:** fix rmw implementation types
- **attachments:** add missing case for attachments
- **check:** add pvc status check
- **column:** fix column name and key
- **discovery-server:** fix robot with non-attached discovery server
- **injections:** fix non-permanent injections to containers
- **ip:** fix broken ip format
- **labels:** :bug: get tenancy labels from robot instead of buildmanager
- **labels:** fix robot image label key
- **launch-manager:** :bug: fix launch pod creation issue
- **path:** fix api path in project file
- **robot-dev-suite:** fix checking robot ide
- **robot-dev-suite:** :bug: fix syncing component specs
- **robot-ide:** update display connection sources
- **status:** fix clearing the workload status if managers are not active
- **status:** clear status from robot
- **typo:** fix vdi resource name

### Pull Requests
- Merge pull request [#24](https://github.com/robolaunch/robot-operator/issues/24) from robolaunch/23-allow-multiple-launches


[Unreleased]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.17...HEAD
[v0.2.5-alpha.17]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.16...v0.2.5-alpha.17
[v0.2.5-alpha.16]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.15...v0.2.5-alpha.16
[v0.2.5-alpha.15]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.13...v0.2.5-alpha.15
[v0.2.5-alpha.13]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.12...v0.2.5-alpha.13
[v0.2.5-alpha.12]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.11...v0.2.5-alpha.12
[v0.2.5-alpha.11]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.10...v0.2.5-alpha.11
[v0.2.5-alpha.10]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.9...v0.2.5-alpha.10
[v0.2.5-alpha.9]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.8...v0.2.5-alpha.9
[v0.2.5-alpha.8]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.6...v0.2.5-alpha.8
[v0.2.5-alpha.6]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.5...v0.2.5-alpha.6
[v0.2.5-alpha.5]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.4...v0.2.5-alpha.5
[v0.2.5-alpha.4]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.3...v0.2.5-alpha.4
[v0.2.5-alpha.3]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.2...v0.2.5-alpha.3
[v0.2.5-alpha.2]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.1...v0.2.5-alpha.2
