# API Reference

## Packages
- [robot.roboscale.io/v1alpha1](#robotroboscaleiov1alpha1)


## robot.roboscale.io/v1alpha1

Package v1alpha1 contains API Schema definitions for the robot v1alpha1 API group

### Resource Types
- [Robot](#robot)
- [BuildManager](#buildmanager)
- [LaunchManager](#launchmanager)
- [RobotDevSuite](#robotdevsuite)
- [DiscoveryServer](#discoveryserver)
- [ROSBridge](#rosbridge)
- [RobotVDI](#robotvdi)
- [RobotIDE](#robotide)



#### Robot



Robot is the Schema for the robots API


| Label Key                              | Description                            |
|----------------------------------------|----------------------------------------|
| `robolaunch.io/organization`           | Name of the organization.              |
| `robolaunch.io/team`                   | Name of the team.                      |
| `robolaunch.io/region`                 | Name of the region.                    |
| `robolaunch.io/cloud-instance`         | Name of the cloud instance.            |
| `robolaunch.io/robot-image-user`       | DockerHub username for custom image.   |
| `robolaunch.io/robot-image-repository` | DockerHub repository for custom image. |
| `robolaunch.io/robot-image-tag`        | DockerHub image tag for custom image.  |


| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `Robot`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[RobotSpec](#robotspec)_ |  |
| `status` _[RobotStatus](#robotstatus)_ |  |


#### RobotSpec



RobotSpec defines the desired state of Robot

_Appears in:_
- [Robot](#robot)

| Field | Description |
| --- | --- |
| `distro` _ROSDistro_ | ROS distro to be used. |
| `storage` _[Storage](#storage)_ | Resource limitations of robot containers. |
| `discoveryServerTemplate` _[DiscoveryServerSpec](#discoveryserverspec)_ | Discovery server template |
| `rosBridgeTemplate` _[ROSBridgeSpec](#rosbridgespec)_ | ROS bridge template |
| `robotDevSuiteTemplate` _[RobotDevSuiteSpec](#robotdevsuitespec)_ | Robot development suite template |
| `workspacesPath` _string_ | Global path of workspaces. It's fixed to `/home/workspaces` path. |
| `workspaces` _[Workspace](#workspace) array_ | Workspace definitions of robot. |
| `development` _boolean_ | Development enabled |
| `rootDNSConfig` _[RootDNSConfig](#rootdnsconfig)_ | Root DNS configuration. |
| `tlsSecretRef` _[TLSSecretReference](#tlssecretreference)_ | TLS secret reference. |


#### RobotStatus



RobotStatus defines the observed state of Robot

_Appears in:_
- [Robot](#robot)

| Field | Description |
| --- | --- |
| `phase` _RobotPhase_ | Phase of robot |
| `image` _string_ | Image of robot |
| `nodeName` _string_ | Node name |
| `volumeStatuses` _[VolumeStatuses](#volumestatuses)_ | Volume status |
| `discoveryServerStatus` _[DiscoveryServerInstanceStatus](#discoveryserverinstancestatus)_ | Discovery server instance status |
| `rosBridgeStatus` _[ROSBridgeInstanceStatus](#rosbridgeinstancestatus)_ | ROS bridge instance status |
| `robotDevSuiteStatus` _[RobotDevSuiteInstanceStatus](#robotdevsuiteinstancestatus)_ | Robot development suite instance status |
| `loaderJobStatus` _[LoaderJobStatus](#loaderjobstatus)_ | Loader job status that configures environment |
| `attachedBuildObject` _[AttachedBuildObject](#attachedbuildobject)_ | Attached build object information |
| `attachedLaunchObjects` _[AttachedLaunchObject](#attachedlaunchobject) array_ | Attached launch object information |
| `attachedDevObjects` _[AttachedDevObject](#attacheddevobject) array_ | Attached dev object information |


#### BuildManager



BuildManager is the Schema for the buildmanagers API

|           Label Key          |        Description        |
|:----------------------------:|:-------------------------:|
| `robolaunch.io/target-robot` | Name of the target robot. |


| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `BuildManager`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[BuildManagerSpec](#buildmanagerspec)_ |  |
| `status` _[BuildManagerStatus](#buildmanagerstatus)_ |  |


#### BuildManagerSpec



BuildManagerSpec defines the desired state of BuildManager

_Appears in:_
- [BuildManager](#buildmanager)

| Field | Description |
| --- | --- |
| `steps` _[Step](#step) array_ |  |


#### BuildManagerStatus



BuildManagerStatus defines the observed state of BuildManager

_Appears in:_
- [AttachedBuildObject](#attachedbuildobject)
- [BuildManager](#buildmanager)

| Field | Description |
| --- | --- |
| `phase` _BuildManagerPhase_ |  |
| `active` _boolean_ |  |
| `scriptConfigMapStatus` _[ScriptConfigMapStatus](#scriptconfigmapstatus)_ |  |
| `steps` _[StepStatus](#stepstatus) array_ |  |


#### LaunchManager



LaunchManager is the Schema for the launchmanagers API


|           Label Key          |        Description        |
|:----------------------------:|:-------------------------:|
| `robolaunch.io/target-robot` | Name of the target robot. |
| `robolaunch.io/target-vdi`   | Name of the target VDI.   |


| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `LaunchManager`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[LaunchManagerSpec](#launchmanagerspec)_ |  |
| `status` _[LaunchManagerStatus](#launchmanagerstatus)_ |  |


#### LaunchManagerSpec



LaunchManagerSpec defines the desired state of LaunchManager

_Appears in:_
- [LaunchManager](#launchmanager)

| Field | Description |
| --- | --- |
| `launch` _object (keys:string, values:[Launch](#launch))_ |  |


#### LaunchManagerStatus



LaunchManagerStatus defines the observed state of LaunchManager

_Appears in:_
- [AttachedLaunchObject](#attachedlaunchobject)
- [LaunchManager](#launchmanager)

| Field | Description |
| --- | --- |
| `phase` _LaunchManagerPhase_ |  |
| `active` _boolean_ |  |
| `launchPodStatus` _[LaunchPodStatus](#launchpodstatus)_ |  |


#### RobotDevSuite



RobotDevSuite is the Schema for the robotdevsuites API


|           Label Key          |        Description        |
|:----------------------------:|:-------------------------:|
| `robolaunch.io/target-robot` | Name of the target robot. |


| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `RobotDevSuite`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[RobotDevSuiteSpec](#robotdevsuitespec)_ |  |
| `status` _[RobotDevSuiteStatus](#robotdevsuitestatus)_ |  |


#### RobotDevSuiteSpec



RobotDevSuiteSpec defines the desired state of RobotDevSuite

_Appears in:_
- [RobotDevSuite](#robotdevsuite)
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `vdiEnabled` _boolean_ |  |
| `robotVDITemplate` _[RobotVDISpec](#robotvdispec)_ |  |
| `ideEnabled` _boolean_ |  |
| `robotIDETemplate` _[RobotIDESpec](#robotidespec)_ |  |


#### RobotDevSuiteStatus



RobotDevSuiteStatus defines the observed state of RobotDevSuite

_Appears in:_
- [AttachedDevObject](#attacheddevobject)
- [RobotDevSuite](#robotdevsuite)
- [RobotDevSuiteInstanceStatus](#robotdevsuiteinstancestatus)

| Field | Description |
| --- | --- |
| `phase` _[RobotDevSuitePhase](#robotdevsuitephase)_ |  |
| `active` _boolean_ |  |
| `robotVDIStatus` _[RobotVDIInstanceStatus](#robotvdiinstancestatus)_ |  |
| `robotIDEStatus` _[RobotIDEInstanceStatus](#robotideinstancestatus)_ |  |


#### DiscoveryServer



DiscoveryServer is the Schema for the discoveryservers API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `DiscoveryServer`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[DiscoveryServerSpec](#discoveryserverspec)_ |  |
| `status` _[DiscoveryServerStatus](#discoveryserverstatus)_ |  |


#### DiscoveryServerSpec



DiscoveryServerSpec defines the desired state of DiscoveryServer

_Appears in:_
- [DiscoveryServer](#discoveryserver)
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `attached` _boolean_ |  |
| `cluster` _string_ |  |
| `hostname` _string_ |  |
| `subdomain` _string_ |  |


#### DiscoveryServerStatus



DiscoveryServerStatus defines the observed state of DiscoveryServer

_Appears in:_
- [DiscoveryServer](#discoveryserver)
- [DiscoveryServerInstanceStatus](#discoveryserverinstancestatus)

| Field | Description |
| --- | --- |
| `phase` _DiscoveryServerPhase_ |  |
| `serviceStatus` _[DiscoveryServerServiceStatus](#discoveryserverservicestatus)_ |  |
| `podStatus` _[DiscoveryServerPodStatus](#discoveryserverpodstatus)_ |  |
| `configMapStatus` _[DiscoveryServerConfigMapStatus](#discoveryserverconfigmapstatus)_ |  |
| `connectionInfo` _[ConnectionInfo](#connectioninfo)_ |  |


#### ROSBridge



ROSBridge is the Schema for the rosbridges API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `ROSBridge`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[ROSBridgeSpec](#rosbridgespec)_ |  |
| `status` _[ROSBridgeStatus](#rosbridgestatus)_ |  |


#### ROSBridgeSpec



ROSBridgeSpec defines the desired state of ROSBridge

_Appears in:_
- [ROSBridge](#rosbridge)
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `ros` _[BridgeDistro](#bridgedistro)_ |  |
| `ros2` _[BridgeDistro](#bridgedistro)_ |  |
| `image` _string_ |  |


#### ROSBridgeStatus



ROSBridgeStatus defines the observed state of ROSBridge

_Appears in:_
- [ROSBridge](#rosbridge)
- [ROSBridgeInstanceStatus](#rosbridgeinstancestatus)

| Field | Description |
| --- | --- |
| `phase` _[BridgePhase](#bridgephase)_ |  |
| `podStatus` _[BridgePodStatus](#bridgepodstatus)_ |  |
| `serviceStatus` _[BridgeServiceStatus](#bridgeservicestatus)_ |  |


#### RobotVDI



RobotVDI is the Schema for the robotvdis API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `RobotVDI`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[RobotVDISpec](#robotvdispec)_ |  |
| `status` _[RobotVDIStatus](#robotvdistatus)_ |  |


#### RobotVDISpec



RobotVDISpec defines the desired state of RobotVDI

_Appears in:_
- [RobotDevSuiteSpec](#robotdevsuitespec)
- [RobotVDI](#robotvdi)

| Field | Description |
| --- | --- |
| `resources` _[Resources](#resources)_ |  |
| `serviceType` _[ServiceType](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#servicetype-v1-core)_ | ServiceType |
| `ingress` _boolean_ |  |
| `privileged` _boolean_ |  |
| `nat1to1` _string_ | NAT1TO1 for Neko. |
| `webrtcPortRange` _string_ |  |


#### RobotVDIStatus



RobotVDIStatus defines the observed state of RobotVDI

_Appears in:_
- [RobotVDI](#robotvdi)

| Field | Description |
| --- | --- |
| `phase` _[RobotVDIPhase](#robotvdiphase)_ |  |
| `podStatus` _[RobotVDIPodStatus](#robotvdipodstatus)_ |  |
| `serviceTCPStatus` _[RobotVDIServiceTCPStatus](#robotvdiservicetcpstatus)_ |  |
| `serviceUDPStatus` _[RobotVDIServiceUDPStatus](#robotvdiserviceudpstatus)_ |  |
| `ingressStatus` _[RobotVDIIngressStatus](#robotvdiingressstatus)_ |  |
| `pvcStatus` _[RobotVDIPVCStatus](#robotvdipvcstatus)_ |  |


#### RobotIDE



RobotIDE is the Schema for the robotides API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `RobotIDE`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[RobotIDESpec](#robotidespec)_ |  |
| `status` _[RobotIDEStatus](#robotidestatus)_ |  |


#### RobotIDESpec



RobotIDESpec defines the desired state of RobotIDE

_Appears in:_
- [RobotDevSuiteSpec](#robotdevsuitespec)
- [RobotIDE](#robotide)

| Field | Description |
| --- | --- |
| `resources` _[Resources](#resources)_ |  |
| `serviceType` _[ServiceType](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#servicetype-v1-core)_ | ServiceType |
| `ingress` _boolean_ |  |
| `privileged` _boolean_ |  |


#### RobotIDEStatus



RobotIDEStatus defines the observed state of RobotIDE

_Appears in:_
- [RobotIDE](#robotide)

| Field | Description |
| --- | --- |
| `phase` _[RobotIDEPhase](#robotidephase)_ |  |
| `podStatus` _[RobotIDEPodStatus](#robotidepodstatus)_ |  |
| `serviceStatus` _[RobotIDEServiceStatus](#robotideservicestatus)_ |  |
| `ingressStatus` _[RobotIDEIngressStatus](#robotideingressstatus)_ |  |


#### AttachedBuildObject





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `reference` _[ObjectReference](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectreference-v1-core)_ |  |
| `status` _[BuildManagerStatus](#buildmanagerstatus)_ |  |


#### AttachedDevObject





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `reference` _[ObjectReference](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectreference-v1-core)_ |  |
| `status` _[RobotDevSuiteStatus](#robotdevsuitestatus)_ |  |


#### AttachedLaunchObject





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `reference` _[ObjectReference](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#objectreference-v1-core)_ |  |
| `status` _[LaunchManagerStatus](#launchmanagerstatus)_ |  |


#### BridgeDistro





_Appears in:_
- [ROSBridgeSpec](#rosbridgespec)

| Field | Description |
| --- | --- |
| `enabled` _boolean_ |  |
| `distro` _ROSDistro_ |  |


#### BridgePhase

_Underlying type:_ `string`



_Appears in:_
- [ROSBridgeStatus](#rosbridgestatus)



#### BridgePodStatus





_Appears in:_
- [ROSBridgeStatus](#rosbridgestatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `phase` _[PodPhase](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#podphase-v1-core)_ |  |


#### BridgeServiceStatus





_Appears in:_
- [ROSBridgeStatus](#rosbridgestatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |


#### ConnectionInfo





_Appears in:_
- [DiscoveryServerStatus](#discoveryserverstatus)

| Field | Description |
| --- | --- |
| `ip` _string_ |  |
| `configMapName` _string_ |  |


#### DiscoveryServerConfigMapStatus





_Appears in:_
- [DiscoveryServerStatus](#discoveryserverstatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |


#### DiscoveryServerInstanceStatus





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `status` _[DiscoveryServerStatus](#discoveryserverstatus)_ |  |


#### DiscoveryServerPodStatus





_Appears in:_
- [DiscoveryServerStatus](#discoveryserverstatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `phase` _[PodPhase](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#podphase-v1-core)_ |  |
| `ip` _string_ |  |


#### DiscoveryServerServiceStatus





_Appears in:_
- [DiscoveryServerStatus](#discoveryserverstatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |


#### JobPhase

_Underlying type:_ `string`



_Appears in:_
- [LoaderJobStatus](#loaderjobstatus)
- [StepStatus](#stepstatus)



#### Launch



Launch description of a repository.

_Appears in:_
- [LaunchManagerSpec](#launchmanagerspec)

| Field | Description |
| --- | --- |
| `selector` _object (keys:string, values:string)_ | Cluster selector. |
| `workspace` _string_ | Name of the workspace. |
| `repository` _string_ | Name of the repository. |
| `namespacing` _boolean_ | Name of the repository. |
| `env` _[EnvVar](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#envvar-v1-core) array_ | Additional environment variables to set when launching ROS nodes. |
| `launchFilePath` _string_ | Path to launchfile in repository. (eg. `linorobot/linorobot_gazebo/launch.py`) |
| `parameters` _object (keys:string, values:string)_ | Launch parameters. |
| `prelaunch` _[Prelaunch](#prelaunch)_ | Command or script to run just before node's execution. |
| `privileged` _boolean_ | Launch container privilege. |


#### LaunchPodStatus





_Appears in:_
- [LaunchManagerStatus](#launchmanagerstatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `phase` _[PodPhase](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#podphase-v1-core)_ |  |
| `ip` _string_ |  |
| `launchStatus` _object (keys:string, values:[LaunchStatus](#launchstatus))_ |  |


#### LaunchStatus





_Appears in:_
- [LaunchPodStatus](#launchpodstatus)

| Field | Description |
| --- | --- |
| `active` _boolean_ |  |
| `containerStatus` _[ContainerStatus](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#containerstatus-v1-core)_ |  |


#### LoaderJobStatus





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `phase` _[JobPhase](#jobphase)_ |  |


#### Prelaunch



Prelaunch command or script is applied just before the node is started.

_Appears in:_
- [Launch](#launch)

| Field | Description |
| --- | --- |
| `command` _string_ | Bash command to run before ROS node execution. |


#### ROSBridgeInstanceStatus





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `status` _[ROSBridgeStatus](#rosbridgestatus)_ |  |


#### Repository



Repository description.

_Appears in:_
- [Workspace](#workspace)

| Field | Description |
| --- | --- |
| `url` _string_ | Base URL of the repository. |
| `branch` _string_ | Branch of the repository to clone. |
| `path` _string_ | [Autofilled] Absolute path of repository |


#### Resources



VDI resource limits.

_Appears in:_
- [RobotIDESpec](#robotidespec)
- [RobotVDISpec](#robotvdispec)

| Field | Description |
| --- | --- |
| `gpuCore` _integer_ |  |
| `cpu` _string_ |  |
| `memory` _string_ |  |


#### RobotDevSuiteInstanceStatus





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `status` _[RobotDevSuiteStatus](#robotdevsuitestatus)_ |  |


#### RobotDevSuitePhase

_Underlying type:_ `string`



_Appears in:_
- [RobotDevSuiteStatus](#robotdevsuitestatus)



#### RobotIDEIngressStatus





_Appears in:_
- [RobotIDEStatus](#robotidestatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |


#### RobotIDEInstanceStatus





_Appears in:_
- [RobotDevSuiteStatus](#robotdevsuitestatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `phase` _[RobotIDEPhase](#robotidephase)_ |  |


#### RobotIDEPhase

_Underlying type:_ `string`



_Appears in:_
- [RobotIDEInstanceStatus](#robotideinstancestatus)
- [RobotIDEStatus](#robotidestatus)



#### RobotIDEPodStatus





_Appears in:_
- [RobotIDEStatus](#robotidestatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `phase` _[PodPhase](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#podphase-v1-core)_ |  |
| `ip` _string_ |  |


#### RobotIDEServiceStatus





_Appears in:_
- [RobotIDEStatus](#robotidestatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |


#### RobotVDIIngressStatus





_Appears in:_
- [RobotVDIStatus](#robotvdistatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |


#### RobotVDIInstanceStatus





_Appears in:_
- [RobotDevSuiteStatus](#robotdevsuitestatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `phase` _[RobotVDIPhase](#robotvdiphase)_ |  |


#### RobotVDIPVCStatus





_Appears in:_
- [RobotVDIStatus](#robotvdistatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |


#### RobotVDIPhase

_Underlying type:_ `string`



_Appears in:_
- [RobotVDIInstanceStatus](#robotvdiinstancestatus)
- [RobotVDIStatus](#robotvdistatus)



#### RobotVDIPodStatus





_Appears in:_
- [RobotVDIStatus](#robotvdistatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `phase` _[PodPhase](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#podphase-v1-core)_ |  |
| `ip` _string_ |  |


#### RobotVDIServiceTCPStatus





_Appears in:_
- [RobotVDIStatus](#robotvdistatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |


#### RobotVDIServiceUDPStatus





_Appears in:_
- [RobotVDIStatus](#robotvdistatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |


#### RootDNSConfig





_Appears in:_
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `host` _string_ | DNS host. |
| `port` _string_ | DNS host. |


#### ScriptConfigMapStatus





_Appears in:_
- [BuildManagerStatus](#buildmanagerstatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |


#### Step



Step is a command or script to execute when building a robot. Either `command` or `script` should be specified for each step.

_Appears in:_
- [BuildManagerSpec](#buildmanagerspec)
- [StepStatus](#stepstatus)

| Field | Description |
| --- | --- |
| `name` _string_ | Name of the step. |
| `workspace` _string_ | Name of the workspace. |
| `command` _string_ | Bash command to run. |
| `script` _string_ | Bash script to run. |
| `env` _[EnvVar](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#envvar-v1-core) array_ | Environment variables for step. |


#### StepStatus





_Appears in:_
- [BuildManagerStatus](#buildmanagerstatus)

| Field | Description |
| --- | --- |
| `step` _[Step](#step)_ |  |
| `jobName` _string_ |  |
| `created` _boolean_ |  |
| `jobPhase` _[JobPhase](#jobphase)_ |  |


#### Storage



Robot's resource limitations.

_Appears in:_
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `amount` _integer_ | Specifies how much storage will be allocated in total. |
| `storageClassConfig` _[StorageClassConfig](#storageclassconfig)_ | Storage class selection for robot's volumes. |


#### StorageClassConfig



Storage class configuration for a volume type.

_Appears in:_
- [Storage](#storage)

| Field | Description |
| --- | --- |
| `name` _string_ | Storage class name |
| `accessMode` _[PersistentVolumeAccessMode](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.25/#persistentvolumeaccessmode-v1-core)_ | PVC access mode |


#### TLSSecretReference





_Appears in:_
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `name` _string_ | TLS secret object name. |
| `namespace` _string_ | TLS secret object namespace. |


#### VolumeStatus





_Appears in:_
- [VolumeStatuses](#volumestatuses)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `persistentVolumeClaimName` _string_ |  |


#### VolumeStatuses





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `var` _[VolumeStatus](#volumestatus)_ |  |
| `etc` _[VolumeStatus](#volumestatus)_ |  |
| `usr` _[VolumeStatus](#volumestatus)_ |  |
| `opt` _[VolumeStatus](#volumestatus)_ |  |
| `workspace` _[VolumeStatus](#volumestatus)_ |  |


#### Workspace



Workspace description. Each robot should contain at least one workspace. A workspace should contain at least one repository in it.

_Appears in:_
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `name` _string_ | Name of workspace. If a workspace's name is `my_ws`, it's absolute path is `/home/workspaces/my_ws`. |
| `repositories` _object (keys:string, values:[Repository](#repository))_ | Repositories to clone inside workspace's `src` directory. |


