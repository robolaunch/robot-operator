<a name="unreleased"></a>
## [Unreleased]

### Fix
- **validation:** fix label existence conditions in dev webhooks


<a name="v0.2.7-alpha.7.1"></a>
## [v0.2.7-alpha.7.1] - 2024-10-03
### Fix
- **labels:** change identical label keys


<a name="v0.2.7-alpha.7"></a>
## [v0.2.7-alpha.7] - 2024-09-11
### Feat
- **host-network:** :rocket: enable host network on request
- **shm-size:** :rocket: make shared mem size selectable


<a name="v0.2.7-alpha.6.4-we-patch-v1"></a>
## [v0.2.7-alpha.6.4-we-patch-v1] - 2024-06-04

<a name="v0.2.7-alpha.6.4"></a>
## [v0.2.7-alpha.6.4] - 2024-03-21
### Feat
- **code-editor:** :rocket: implement code editor (robotide in v1alpha1)

### Fix
- **update:** remove already exists condition from update to prevent redundant reconciles


<a name="v0.2.7-alpha.6.3"></a>
## [v0.2.7-alpha.6.3] - 2024-03-18

<a name="v0.2.7-alpha.6.2"></a>
## [v0.2.7-alpha.6.2] - 2024-03-18
### Fix
- **protocol:** switch protocol field from v1.Protocol to string


<a name="v0.2.7-alpha.6.1"></a>
## [v0.2.7-alpha.6.1] - 2024-03-07

<a name="v0.2.7-alpha.6"></a>
## [v0.2.7-alpha.6] - 2024-03-06
### Feat
- **discovery-server:** :rocket: select protocol for discovery server

### Fix
- **typo:** fix field reference


<a name="v0.2.7-alpha.4-udp-test-2"></a>
## [v0.2.7-alpha.4-udp-test-2] - 2024-03-04

<a name="v0.2.7-alpha.5"></a>
## [v0.2.7-alpha.5] - 2024-03-04

<a name="v0.2.7-alpha.4"></a>
## [v0.2.7-alpha.4] - 2024-02-15

<a name="v0.2.7-alpha.1"></a>
## [v0.2.7-alpha.1] - 2024-02-05

<a name="v0.2.6-alpha.19"></a>
## [v0.2.6-alpha.19] - 2024-01-29

<a name="v0.2.6-alpha.18"></a>
## [v0.2.6-alpha.18] - 2024-01-26
### Fix
- **gpu:** set gpu core count to 0 if the service is not enabled


<a name="v0.2.6-alpha.17"></a>
## [v0.2.6-alpha.17] - 2024-01-24
### Feat
- **launch:** integrate launchmanager for environments


<a name="v0.2.6-alpha.16"></a>
## [v0.2.6-alpha.16] - 2024-01-19
### Feat
- **build:** integrate buildmanager for environments


<a name="v0.2.6-alpha.15"></a>
## [v0.2.6-alpha.15] - 2024-01-18
### Fix
- **notebook:** :bug: allow notebook connections over ingress


<a name="v0.2.6-alpha.14.9"></a>
## [v0.2.6-alpha.14.9] - 2024-01-02
### Fix
- **service:** update connection keys


<a name="v0.2.6-alpha.14.8"></a>
## [v0.2.6-alpha.14.8] - 2024-01-02
### Feat
- **notebook:** integrate jupyter notebook

### Fix
- **service:** update service path


<a name="v0.2.6-alpha.14.7"></a>
## [v0.2.6-alpha.14.7] - 2023-12-25

<a name="v0.2.6-alpha.14.6"></a>
## [v0.2.6-alpha.14.6] - 2023-12-13
### Fix
- **filebrowser:** add base url to the file browser
- **filebrowser:** add file browser base url as env variable to services


<a name="v0.2.6-alpha.14.5"></a>
## [v0.2.6-alpha.14.5] - 2023-12-13

<a name="v0.2.6-alpha.14.4"></a>
## [v0.2.6-alpha.14.4] - 2023-12-13
### Fix
- **cm:** update cm status of robotide


<a name="v0.2.6-alpha.14.3"></a>
## [v0.2.6-alpha.14.3] - 2023-12-13

<a name="v0.2.6-alpha.14.2"></a>
## [v0.2.6-alpha.14.2] - 2023-12-06

<a name="v0.2.6-alpha.14.1"></a>
## [v0.2.6-alpha.14.1] - 2023-12-05
### Feat
- **background:** support custom background processes in ide


<a name="v0.2.6-alpha.14"></a>
## [v0.2.6-alpha.14] - 2023-12-04
### Feat
- **dcgm:** get gpu device information from dcgm
- **file-browser:** support file browser in robots/applications

### Fix
- **api:** convert gpu metrics from string to string map
- **dockerfile:** update dockerfile for custom metrics patcher
- **script:** reassign old metric key
- **script:** fix typos in fields and update dockerfile


<a name="v0.2.6-alpha.13.3"></a>
## [v0.2.6-alpha.13.3] - 2023-11-20

<a name="v0.2.6-alpha.13.2-hostnetwork-enabled"></a>
## [v0.2.6-alpha.13.2-hostnetwork-enabled] - 2023-11-16

<a name="v0.2.6-alpha.13.1"></a>
## [v0.2.6-alpha.13.1] - 2023-11-09

<a name="v0.2.6-alpha.13"></a>
## [v0.2.6-alpha.13] - 2023-11-08
### Feat
- **storage:** :rocket: track storage/fs usages w/ metrics exporter
- **volumes:** :rocket: mount volume from host


<a name="v0.2.6-alpha.12.2"></a>
## [v0.2.6-alpha.12.2] - 2023-11-07

<a name="v0.2.6-alpha.12.1"></a>
## [v0.2.6-alpha.12.1] - 2023-11-02

<a name="v0.2.6-alpha.12"></a>
## [v0.2.6-alpha.12] - 2023-10-30

<a name="v0.2.6-alpha.11"></a>
## [v0.2.6-alpha.11] - 2023-10-30
### Feat
- **gpu:** :rocket: track gpu allocations w/ metrics exporter

### Fix
- **gpu-metrics:** set gpu instance as nvidia.com/gpu by default


<a name="v0.2.6-alpha.10"></a>
## [v0.2.6-alpha.10] - 2023-10-17
### Feat
- **ports:** :rocket: specify custom ports for ide and vdi container


<a name="v0.2.6-alpha.9"></a>
## [v0.2.6-alpha.9] - 2023-10-16
### Feat
- **persistency:** :rocket: enable choosing directories to make them persistent

### Fix
- **injections:** fix func arguments for ipp injection


<a name="v0.2.6-alpha.8"></a>
## [v0.2.6-alpha.8] - 2023-09-26
### Fix
- **offline:** :bug: ignore cloning errors in offline mode


<a name="v0.2.6-alpha.7"></a>
## [v0.2.6-alpha.7] - 2023-09-25
### Feat
- **gpu:** :rocket: support additional configs

### Fix
- **env:** :bug: replace containers w/ initcontainers in env config injection


<a name="v0.2.6-alpha.6"></a>
## [v0.2.6-alpha.6] - 2023-09-22
### Fix
- **image-pull-policy:** :bug: set ipp of initcontainers


<a name="v0.2.6-alpha.5"></a>
## [v0.2.6-alpha.5] - 2023-09-18

<a name="v0.2.6-alpha.4"></a>
## [v0.2.6-alpha.4] - 2023-09-08
### Fix
- **webhooks:** set gpu instance type if it's not specified


<a name="v0.2.6-alpha.3"></a>
## [v0.2.6-alpha.3] - 2023-09-07
### Feat
- **gpu:** :rocket: support mig instances


<a name="v0.2.6-alpha.2"></a>
## [v0.2.6-alpha.2] - 2023-08-30
### Feat
- **image:** :rocket: use config map for image query
- **registry:** :rocket: support custom image registries
- **vdi:** :rocket: add cpu option


<a name="v0.2.6-alpha.1"></a>
## [v0.2.6-alpha.1] - 2023-08-10
### Feat
- **environment:** :rocket: support provisioning environments
- **environment:** :rocket: implement environment api

### Fix
- **webhooks:** :bug: update webhook condition


<a name="v0.2.5-alpha.34"></a>
## [v0.2.5-alpha.34] - 2023-08-09

<a name="v0.2.5-alpha.33"></a>
## [v0.2.5-alpha.33] - 2023-08-09
### Fix
- **bridge:** disable ros bridge on demand
- **controllers:** handle cyclomatic complexities in controllers


<a name="v0.2.5-alpha.32"></a>
## [v0.2.5-alpha.32] - 2023-08-07

<a name="v0.2.5-alpha.31"></a>
## [v0.2.5-alpha.31] - 2023-08-07

<a name="v0.2.5-alpha.30"></a>
## [v0.2.5-alpha.30] - 2023-08-02
### Feat
- **domain-id:** :rocket: support ros domain id

### Fix
- **workspaces:** set default workspace path


<a name="v0.2.5-alpha.29"></a>
## [v0.2.5-alpha.29] - 2023-07-20
### Fix
- **launch:** pass environment variables to launch container
- **launch-manager:** :bug: update launch container


<a name="v0.2.5-alpha.28"></a>
## [v0.2.5-alpha.28] - 2023-07-14
### Fix
- **ingress:** define keys and values for annotations
- **websocket:** avoid websocket connection interrupts in tcp ws services


<a name="v0.2.5-alpha.27"></a>
## [v0.2.5-alpha.27] - 2023-07-12

<a name="v0.2.5-alpha.26"></a>
## [v0.2.5-alpha.26] - 2023-07-11
### Fix
- **nodeport:** replace internal ports connection prefixes w/ node ports
- **nodeport:** add connection prefixes


<a name="v0.2.5-alpha.25"></a>
## [v0.2.5-alpha.25] - 2023-07-11
### Fix
- **nodeport:** fix endpoint routes


<a name="v0.2.5-alpha.24"></a>
## [v0.2.5-alpha.24] - 2023-07-11

<a name="v0.2.5-alpha.23"></a>
## [v0.2.5-alpha.23] - 2023-07-07
### Fix
- **discoveryserver:** check discovery server ip frequently from client objects


<a name="v0.2.5-alpha.22"></a>
## [v0.2.5-alpha.22] - 2023-07-07
### Fix
- **serviceexport:** check robotide serviceexport resource status


<a name="v0.2.5-alpha.21"></a>
## [v0.2.5-alpha.21] - 2023-07-04
### Fix
- **remote-ide:** update rds operational conditions


<a name="v0.2.5-alpha.20"></a>
## [v0.2.5-alpha.20] - 2023-06-23
### Feat
- **relayserver:** :tada: use relay server to serve remote services
- **remote:** :rocket: make remote ide consumable from cloud instance

### Fix
- **webhooks:** :bug: disable relay server webhooks and enable robotdevsuite webhooks


<a name="v0.2.5-alpha.19"></a>
## [v0.2.5-alpha.19] - 2023-06-21
### Fix
- **security-context:** :bug: remove checking container privilege for ws manager jobs


<a name="v0.2.5-alpha.18"></a>
## [v0.2.5-alpha.18] - 2023-06-21
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


<a name="v0.2.5-alpha.7"></a>
## [v0.2.5-alpha.7] - 2023-04-24

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


[Unreleased]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.7.1...HEAD
[v0.2.7-alpha.7.1]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.7...v0.2.7-alpha.7.1
[v0.2.7-alpha.7]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.6.4-we-patch-v1...v0.2.7-alpha.7
[v0.2.7-alpha.6.4-we-patch-v1]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.6.4...v0.2.7-alpha.6.4-we-patch-v1
[v0.2.7-alpha.6.4]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.6.3...v0.2.7-alpha.6.4
[v0.2.7-alpha.6.3]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.6.2...v0.2.7-alpha.6.3
[v0.2.7-alpha.6.2]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.6.1...v0.2.7-alpha.6.2
[v0.2.7-alpha.6.1]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.6...v0.2.7-alpha.6.1
[v0.2.7-alpha.6]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.4-udp-test-2...v0.2.7-alpha.6
[v0.2.7-alpha.4-udp-test-2]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.5...v0.2.7-alpha.4-udp-test-2
[v0.2.7-alpha.5]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.4...v0.2.7-alpha.5
[v0.2.7-alpha.4]: https://github.com/robolaunch/robot-operator/compare/v0.2.7-alpha.1...v0.2.7-alpha.4
[v0.2.7-alpha.1]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.19...v0.2.7-alpha.1
[v0.2.6-alpha.19]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.18...v0.2.6-alpha.19
[v0.2.6-alpha.18]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.17...v0.2.6-alpha.18
[v0.2.6-alpha.17]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.16...v0.2.6-alpha.17
[v0.2.6-alpha.16]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.15...v0.2.6-alpha.16
[v0.2.6-alpha.15]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.14.9...v0.2.6-alpha.15
[v0.2.6-alpha.14.9]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.14.8...v0.2.6-alpha.14.9
[v0.2.6-alpha.14.8]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.14.7...v0.2.6-alpha.14.8
[v0.2.6-alpha.14.7]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.14.6...v0.2.6-alpha.14.7
[v0.2.6-alpha.14.6]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.14.5...v0.2.6-alpha.14.6
[v0.2.6-alpha.14.5]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.14.4...v0.2.6-alpha.14.5
[v0.2.6-alpha.14.4]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.14.3...v0.2.6-alpha.14.4
[v0.2.6-alpha.14.3]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.14.2...v0.2.6-alpha.14.3
[v0.2.6-alpha.14.2]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.14.1...v0.2.6-alpha.14.2
[v0.2.6-alpha.14.1]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.14...v0.2.6-alpha.14.1
[v0.2.6-alpha.14]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.13.3...v0.2.6-alpha.14
[v0.2.6-alpha.13.3]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.13.2-hostnetwork-enabled...v0.2.6-alpha.13.3
[v0.2.6-alpha.13.2-hostnetwork-enabled]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.13.1...v0.2.6-alpha.13.2-hostnetwork-enabled
[v0.2.6-alpha.13.1]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.13...v0.2.6-alpha.13.1
[v0.2.6-alpha.13]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.12.2...v0.2.6-alpha.13
[v0.2.6-alpha.12.2]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.12.1...v0.2.6-alpha.12.2
[v0.2.6-alpha.12.1]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.12...v0.2.6-alpha.12.1
[v0.2.6-alpha.12]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.11...v0.2.6-alpha.12
[v0.2.6-alpha.11]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.10...v0.2.6-alpha.11
[v0.2.6-alpha.10]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.9...v0.2.6-alpha.10
[v0.2.6-alpha.9]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.8...v0.2.6-alpha.9
[v0.2.6-alpha.8]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.7...v0.2.6-alpha.8
[v0.2.6-alpha.7]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.6...v0.2.6-alpha.7
[v0.2.6-alpha.6]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.5...v0.2.6-alpha.6
[v0.2.6-alpha.5]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.4...v0.2.6-alpha.5
[v0.2.6-alpha.4]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.3...v0.2.6-alpha.4
[v0.2.6-alpha.3]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.2...v0.2.6-alpha.3
[v0.2.6-alpha.2]: https://github.com/robolaunch/robot-operator/compare/v0.2.6-alpha.1...v0.2.6-alpha.2
[v0.2.6-alpha.1]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.34...v0.2.6-alpha.1
[v0.2.5-alpha.34]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.33...v0.2.5-alpha.34
[v0.2.5-alpha.33]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.32...v0.2.5-alpha.33
[v0.2.5-alpha.32]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.31...v0.2.5-alpha.32
[v0.2.5-alpha.31]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.30...v0.2.5-alpha.31
[v0.2.5-alpha.30]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.29...v0.2.5-alpha.30
[v0.2.5-alpha.29]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.28...v0.2.5-alpha.29
[v0.2.5-alpha.28]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.27...v0.2.5-alpha.28
[v0.2.5-alpha.27]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.26...v0.2.5-alpha.27
[v0.2.5-alpha.26]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.25...v0.2.5-alpha.26
[v0.2.5-alpha.25]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.24...v0.2.5-alpha.25
[v0.2.5-alpha.24]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.23...v0.2.5-alpha.24
[v0.2.5-alpha.23]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.22...v0.2.5-alpha.23
[v0.2.5-alpha.22]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.21...v0.2.5-alpha.22
[v0.2.5-alpha.21]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.20...v0.2.5-alpha.21
[v0.2.5-alpha.20]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.19...v0.2.5-alpha.20
[v0.2.5-alpha.19]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.18...v0.2.5-alpha.19
[v0.2.5-alpha.18]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.17...v0.2.5-alpha.18
[v0.2.5-alpha.17]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.16...v0.2.5-alpha.17
[v0.2.5-alpha.16]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.15...v0.2.5-alpha.16
[v0.2.5-alpha.15]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.13...v0.2.5-alpha.15
[v0.2.5-alpha.13]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.12...v0.2.5-alpha.13
[v0.2.5-alpha.12]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.11...v0.2.5-alpha.12
[v0.2.5-alpha.11]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.10...v0.2.5-alpha.11
[v0.2.5-alpha.10]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.9...v0.2.5-alpha.10
[v0.2.5-alpha.9]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.8...v0.2.5-alpha.9
[v0.2.5-alpha.8]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.7...v0.2.5-alpha.8
[v0.2.5-alpha.7]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.6...v0.2.5-alpha.7
[v0.2.5-alpha.6]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.5...v0.2.5-alpha.6
[v0.2.5-alpha.5]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.4...v0.2.5-alpha.5
[v0.2.5-alpha.4]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.3...v0.2.5-alpha.4
[v0.2.5-alpha.3]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.2...v0.2.5-alpha.3
[v0.2.5-alpha.2]: https://github.com/robolaunch/robot-operator/compare/v0.2.5-alpha.1...v0.2.5-alpha.2
