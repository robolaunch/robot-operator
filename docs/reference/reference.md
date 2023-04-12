# API Reference


## robot.roboscale.io/v1alpha1

Package v1alpha1 contains API Schema definitions for the robot v1alpha1 API group

### Resource Types
- [Robot](#robot)
- [WorkspaceManager](#workspacemanager)
- [BuildManager](#buildmanager)
- [LaunchManager](#launchmanager)
- [RobotDevSuite](#robotdevsuite)
- [MetricsExporter](#metricsexporter)
- [RobotVDI](#robotvdi)
- [RobotIDE](#robotide)
- [DiscoveryServer](#discoveryserver)
- [ROSBridge](#rosbridge)
- [RobotArtifact](#robotartifact)



#### Robot



Robot is the Schema for the robots API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `Robot`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[RobotSpec](#robotspec)_ |  |
| `status` _[RobotStatus](#robotstatus)_ |  |


#### RobotSpec



RobotSpec defines the desired state of Robot

_Appears in:_
- [Robot](#robot)
- [RobotArtifact](#robotartifact)

| Field | Description |
| --- | --- |
| `distributions` _[ROSDistro](#rosdistro) array_ | ROS distro to be used. |
| `rmwImplementation` _[RMWImplementation](#rmwimplementation)_ |  |
| `storage` _[Storage](#storage)_ | Resource limitations of robot containers. |
| `discoveryServerTemplate` _[DiscoveryServerSpec](#discoveryserverspec)_ | Discovery server template |
| `rosBridgeTemplate` _[ROSBridgeSpec](#rosbridgespec)_ | ROS bridge template |
| `workspaceManagerTemplate` _[WorkspaceManagerSpec](#workspacemanagerspec)_ | Workspace manager template |
| `buildManagerTemplate` _[BuildManagerSpec](#buildmanagerspec)_ | Build manager template for initial configuration |
| `launchManagerTemplates` _[LaunchManagerSpec](#launchmanagerspec) array_ | Launch manager template for initial configuration |
| `robotDevSuiteTemplate` _[RobotDevSuiteSpec](#robotdevsuitespec)_ | Robot development suite template |
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
| `loaderJobStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ | Loader job status that configures environment |
| `workspaceManagerStatus` _[WorkspaceManagerInstanceStatus](#workspacemanagerinstancestatus)_ | Workspace manager status |
| `initialBuildManagerStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ | Initial build manager creation status |
| `initialLaunchManagerStatuses` _[OwnedResourceStatus](#ownedresourcestatus) array_ | Initial launch manager creation status |
| `attachedBuildObject` _[AttachedBuildObject](#attachedbuildobject)_ | Attached build object information |
| `attachedLaunchObjects` _[AttachedLaunchObject](#attachedlaunchobject) array_ | Attached launch object information |
| `attachedDevObjects` _[AttachedDevObject](#attacheddevobject) array_ | Attached dev object information |


#### WorkspaceManager



WorkspaceManager is the Schema for the workspacemanagers API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `WorkspaceManager`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[WorkspaceManagerSpec](#workspacemanagerspec)_ |  |
| `status` _[WorkspaceManagerStatus](#workspacemanagerstatus)_ |  |


#### WorkspaceManagerSpec



WorkspaceManagerSpec defines the desired state of WorkspaceManager

_Appears in:_
- [RobotSpec](#robotspec)
- [WorkspaceManager](#workspacemanager)

| Field | Description |
| --- | --- |
| `workspacesPath` _string_ | Global path of workspaces. It's fixed to `/home/workspaces` path. |
| `workspaces` _[Workspace](#workspace) array_ | Workspace definitions of robot. |
| `updateNeeded` _boolean_ | Need update |


#### WorkspaceManagerStatus



WorkspaceManagerStatus defines the observed state of WorkspaceManager

_Appears in:_
- [WorkspaceManager](#workspacemanager)
- [WorkspaceManagerInstanceStatus](#workspacemanagerinstancestatus)

| Field | Description |
| --- | --- |
| `phase` _[WorkspaceManagerPhase](#workspacemanagerphase)_ |  |
| `clonerJobStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `cleanupJobStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `version` _integer_ |  |


#### BuildManager



BuildManager is the Schema for the buildmanagers API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `BuildManager`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[BuildManagerSpec](#buildmanagerspec)_ |  |
| `status` _[BuildManagerStatus](#buildmanagerstatus)_ |  |


#### BuildManagerSpec



BuildManagerSpec defines the desired state of BuildManager

_Appears in:_
- [BuildManager](#buildmanager)
- [RobotSpec](#robotspec)

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
| `phase` _[BuildManagerPhase](#buildmanagerphase)_ |  |
| `active` _boolean_ |  |
| `scriptConfigMapStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `steps` _[StepStatus](#stepstatus) array_ |  |


#### LaunchManager



LaunchManager is the Schema for the launchmanagers API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `LaunchManager`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[LaunchManagerSpec](#launchmanagerspec)_ |  |
| `status` _[LaunchManagerStatus](#launchmanagerstatus)_ |  |


#### LaunchManagerSpec



LaunchManagerSpec defines the desired state of LaunchManager

_Appears in:_
- [LaunchManager](#launchmanager)
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `display` _boolean_ | Display connection. |
| `launch` _object (keys:string, values:[Launch](#launch))_ |  |
| `run` _object (keys:string, values:[Run](#run))_ |  |


#### LaunchManagerStatus



LaunchManagerStatus defines the observed state of LaunchManager

_Appears in:_
- [AttachedLaunchObject](#attachedlaunchobject)
- [LaunchManager](#launchmanager)

| Field | Description |
| --- | --- |
| `phase` _[LaunchManagerPhase](#launchmanagerphase)_ |  |
| `active` _boolean_ |  |
| `launchPodStatus` _[LaunchPodStatus](#launchpodstatus)_ |  |


#### RobotDevSuite



RobotDevSuite is the Schema for the robotdevsuites API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `RobotDevSuite`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
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
| `robotVDIStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `robotIDEStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |


#### MetricsExporter



MetricsExporter is the Schema for the metricsexporters API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `MetricsExporter`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[MetricsExporterSpec](#metricsexporterspec)_ |  |
| `status` _[MetricsExporterStatus](#metricsexporterstatus)_ |  |


#### MetricsExporterSpec



MetricsExporterSpec defines the desired state of MetricsExporter

_Appears in:_
- [MetricsExporter](#metricsexporter)

| Field | Description |
| --- | --- |
| `gpu` _[GPUMetrics](#gpumetrics)_ |  |
| `network` _[NetworkMetrics](#networkmetrics)_ |  |


#### MetricsExporterStatus



MetricsExporterStatus defines the observed state of MetricsExporter

_Appears in:_
- [MetricsExporter](#metricsexporter)

| Field | Description |
| --- | --- |
| `phase` _[MetricsExporterPhase](#metricsexporterphase)_ |  |
| `roleStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `roleBindingStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `saStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `podStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `usage` _[Usage](#usage)_ |  |


#### RobotVDI



RobotVDI is the Schema for the robotvdis API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `RobotVDI`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
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
| `serviceType` _[ServiceType](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#servicetype-v1-core)_ | ServiceType |
| `ingress` _boolean_ |  |
| `privileged` _boolean_ |  |
| `nat1to1` _string_ | NAT1TO1 for Neko. |
| `webrtcPortRange` _string_ |  |
| `resolution` _string_ | VDI screen resolution options. |


#### RobotVDIStatus



RobotVDIStatus defines the observed state of RobotVDI

_Appears in:_
- [RobotVDI](#robotvdi)

| Field | Description |
| --- | --- |
| `phase` _[RobotVDIPhase](#robotvdiphase)_ |  |
| `podStatus` _[OwnedPodStatus](#ownedpodstatus)_ |  |
| `serviceTCPStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `serviceUDPStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `ingressStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `pvcStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |


#### RobotIDE



RobotIDE is the Schema for the robotides API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `RobotIDE`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
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
| `serviceType` _[ServiceType](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#servicetype-v1-core)_ | ServiceType |
| `ingress` _boolean_ |  |
| `privileged` _boolean_ |  |
| `display` _boolean_ | Display configuration. |


#### RobotIDEStatus



RobotIDEStatus defines the observed state of RobotIDE

_Appears in:_
- [RobotIDE](#robotide)

| Field | Description |
| --- | --- |
| `phase` _[RobotIDEPhase](#robotidephase)_ |  |
| `podStatus` _[OwnedPodStatus](#ownedpodstatus)_ |  |
| `serviceStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `ingressStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |


#### DiscoveryServer



DiscoveryServer is the Schema for the discoveryservers API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `DiscoveryServer`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[DiscoveryServerSpec](#discoveryserverspec)_ |  |
| `status` _[DiscoveryServerStatus](#discoveryserverstatus)_ |  |


#### DiscoveryServerSpec



DiscoveryServerSpec defines the desired state of DiscoveryServer

_Appears in:_
- [DiscoveryServer](#discoveryserver)
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `type` _[DiscoveryServerInstanceType](#discoveryserverinstancetype)_ |  |
| `reference` _[ObjectReference](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectreference-v1-core)_ |  |
| `cluster` _string_ |  |
| `hostname` _string_ |  |
| `subdomain` _string_ |  |
| `image` _string_ |  |
| `args` _string array_ |  |


#### DiscoveryServerStatus



DiscoveryServerStatus defines the observed state of DiscoveryServer

_Appears in:_
- [DiscoveryServer](#discoveryserver)
- [DiscoveryServerInstanceStatus](#discoveryserverinstancestatus)

| Field | Description |
| --- | --- |
| `phase` _DiscoveryServerPhase_ |  |
| `serviceStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `serviceExportStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `podStatus` _[OwnedPodStatus](#ownedpodstatus)_ |  |
| `configMapStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `connectionInfo` _[ConnectionInfo](#connectioninfo)_ |  |


#### ROSBridge



ROSBridge is the Schema for the rosbridges API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `ROSBridge`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
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
| `phase` _BridgePhase_ |  |
| `podStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `serviceStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |


#### RobotArtifact



RobotArtifact is the Schema for the robotartifacts API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha1`
| `kind` _string_ | `RobotArtifact`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `template` _[RobotSpec](#robotspec)_ |  |


#### AttachedBuildObject





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `reference` _[ObjectReference](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectreference-v1-core)_ |  |
| `status` _[BuildManagerStatus](#buildmanagerstatus)_ |  |


#### AttachedDevObject





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `reference` _[ObjectReference](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectreference-v1-core)_ |  |
| `status` _[RobotDevSuiteStatus](#robotdevsuitestatus)_ |  |


#### AttachedLaunchObject





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `reference` _[ObjectReference](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectreference-v1-core)_ |  |
| `status` _[LaunchManagerStatus](#launchmanagerstatus)_ |  |


#### BridgeDistro





_Appears in:_
- [ROSBridgeSpec](#rosbridgespec)

| Field | Description |
| --- | --- |
| `enabled` _boolean_ |  |
| `distro` _[ROSDistro](#rosdistro)_ |  |


#### BuildManagerPhase

_Underlying type:_ `string`



_Appears in:_
- [BuildManagerStatus](#buildmanagerstatus)



#### ConnectionInfo





_Appears in:_
- [DiscoveryServerStatus](#discoveryserverstatus)

| Field | Description |
| --- | --- |
| `ip` _string_ |  |
| `configMapName` _string_ |  |


#### DiscoveryServerInstanceStatus





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `status` _[DiscoveryServerStatus](#discoveryserverstatus)_ |  |


#### DiscoveryServerInstanceType

_Underlying type:_ `string`



_Appears in:_
- [DiscoveryServerSpec](#discoveryserverspec)



#### GPUMetrics





_Appears in:_
- [MetricsExporterSpec](#metricsexporterspec)

| Field | Description |
| --- | --- |
| `track` _boolean_ |  |
| `interval` _integer_ |  |


#### GPUUtilizationStatus





_Appears in:_
- [Usage](#usage)

| Field | Description |
| --- | --- |
| `utilization` _string_ |  |
| `lastUpdateTimestamp` _string_ |  |


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
| `env` _[EnvVar](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#envvar-v1-core) array_ | Additional environment variables to set when launching ROS nodes. |
| `launchFilePath` _string_ | Path to launchfile in repository. (eg. `linorobot/linorobot_gazebo/launch.py`) |
| `parameters` _object (keys:string, values:string)_ | Launch parameters. |
| `prelaunch` _[Prelaunch](#prelaunch)_ | Command or script to run just before node's execution. |
| `privileged` _boolean_ | Launch container privilege. |
| `resources` _[Resources](#resources)_ | Launch container resource limits. |


#### LaunchManagerPhase

_Underlying type:_ `string`



_Appears in:_
- [LaunchManagerStatus](#launchmanagerstatus)



#### LaunchPodStatus





_Appears in:_
- [LaunchManagerStatus](#launchmanagerstatus)

| Field | Description |
| --- | --- |
| `status` _[OwnedPodStatus](#ownedpodstatus)_ |  |
| `launchStatus` _object (keys:string, values:[LaunchStatus](#launchstatus))_ |  |


#### LaunchStatus





_Appears in:_
- [LaunchPodStatus](#launchpodstatus)

| Field | Description |
| --- | --- |
| `active` _boolean_ |  |
| `containerStatus` _[ContainerStatus](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#containerstatus-v1-core)_ |  |


#### MetricsExporterPhase

_Underlying type:_ `string`



_Appears in:_
- [MetricsExporterStatus](#metricsexporterstatus)



#### NetworkInterfaceLoad





_Appears in:_
- [NetworkLoadStatus](#networkloadstatus)

| Field | Description |
| --- | --- |
| `in` _string_ |  |
| `out` _string_ |  |


#### NetworkLoadStatus





_Appears in:_
- [Usage](#usage)

| Field | Description |
| --- | --- |
| `load` _object (keys:string, values:[NetworkInterfaceLoad](#networkinterfaceload))_ |  |
| `lastUpdateTimestamp` _string_ |  |


#### NetworkMetrics





_Appears in:_
- [MetricsExporterSpec](#metricsexporterspec)

| Field | Description |
| --- | --- |
| `track` _boolean_ |  |
| `interval` _integer_ |  |
| `interfaces` _string array_ |  |


#### OwnedPodStatus





_Appears in:_
- [DiscoveryServerStatus](#discoveryserverstatus)
- [LaunchPodStatus](#launchpodstatus)
- [RobotIDEStatus](#robotidestatus)
- [RobotVDIStatus](#robotvdistatus)

| Field | Description |
| --- | --- |
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `ip` _string_ |  |


#### OwnedResourceStatus





_Appears in:_
- [BuildManagerStatus](#buildmanagerstatus)
- [DiscoveryServerInstanceStatus](#discoveryserverinstancestatus)
- [DiscoveryServerStatus](#discoveryserverstatus)
- [MetricsExporterStatus](#metricsexporterstatus)
- [OwnedPodStatus](#ownedpodstatus)
- [ROSBridgeInstanceStatus](#rosbridgeinstancestatus)
- [ROSBridgeStatus](#rosbridgestatus)
- [RobotDevSuiteInstanceStatus](#robotdevsuiteinstancestatus)
- [RobotDevSuiteStatus](#robotdevsuitestatus)
- [RobotIDEStatus](#robotidestatus)
- [RobotStatus](#robotstatus)
- [RobotVDIStatus](#robotvdistatus)
- [StepStatus](#stepstatus)
- [VolumeStatuses](#volumestatuses)
- [WorkspaceManagerInstanceStatus](#workspacemanagerinstancestatus)
- [WorkspaceManagerStatus](#workspacemanagerstatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ |  |
| `reference` _[ObjectReference](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectreference-v1-core)_ |  |
| `phase` _string_ |  |


#### Prelaunch



Prelaunch command or script is applied just before the node is started.

_Appears in:_
- [Launch](#launch)
- [Run](#run)

| Field | Description |
| --- | --- |
| `command` _string_ | Bash command to run before ROS node execution. |


#### RMWImplementation

_Underlying type:_ `string`

RMW implementation selection. Robot operator currently supports only FastRTPS. See https://docs.ros.org/en/foxy/How-To-Guides/Working-with-multiple-RMW-implementations.html.

_Appears in:_
- [RobotSpec](#robotspec)



#### ROSBridgeInstanceStatus





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `status` _[ROSBridgeStatus](#rosbridgestatus)_ |  |


#### ROSDistro

_Underlying type:_ `string`

ROS distro selection. Allowed distros are Foxy and Galactic. It is aimed to support Humble, Melodic and Noetic in further versions.

_Appears in:_
- [BridgeDistro](#bridgedistro)
- [RobotSpec](#robotspec)
- [Workspace](#workspace)



#### Repository



Repository description.

_Appears in:_
- [Workspace](#workspace)

| Field | Description |
| --- | --- |
| `url` _string_ | Base URL of the repository. |
| `branch` _string_ | Branch of the repository to clone. |
| `path` _string_ | [Autofilled] Absolute path of repository |
| `owner` _string_ | [Autofilled] User or organization, maintainer of repository |
| `repo` _string_ | [Autofilled] Repository name |
| `hash` _string_ | [Autofilled] Hash of last commit |


#### Resources



VDI resource limits.

_Appears in:_
- [Launch](#launch)
- [RobotIDESpec](#robotidespec)
- [RobotVDISpec](#robotvdispec)
- [Run](#run)

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
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `status` _[RobotDevSuiteStatus](#robotdevsuitestatus)_ |  |


#### RobotDevSuitePhase

_Underlying type:_ `string`



_Appears in:_
- [RobotDevSuiteStatus](#robotdevsuitestatus)



#### RobotIDEPhase

_Underlying type:_ `string`



_Appears in:_
- [RobotIDEStatus](#robotidestatus)



#### RobotVDIPhase

_Underlying type:_ `string`



_Appears in:_
- [RobotVDIStatus](#robotvdistatus)



#### RootDNSConfig





_Appears in:_
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `host` _string_ | DNS host. |


#### Run



Run description.

_Appears in:_
- [LaunchManagerSpec](#launchmanagerspec)

| Field | Description |
| --- | --- |
| `selector` _object (keys:string, values:string)_ | Cluster selector. |
| `workspace` _string_ | Name of the workspace. |
| `namespacing` _boolean_ | Name of the repository. |
| `env` _[EnvVar](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#envvar-v1-core) array_ | Additional environment variables to set when launching ROS nodes. |
| `package` _string_ | Package name in `ros2 run <package> <executable>`. |
| `executable` _string_ | Executable name in `ros2 run <package> <executable>`. |
| `parameters` _object (keys:string, values:string)_ | Launch parameters. |
| `prelaunch` _[Prelaunch](#prelaunch)_ | Command or script to run just before node's execution. |
| `privileged` _boolean_ | Launch container privilege. |
| `resources` _[Resources](#resources)_ | Launch container resource limits. |


#### Step



Step is a command or script to execute when building a robot. Either `command` or `script` should be specified for each step.

_Appears in:_
- [BuildManagerSpec](#buildmanagerspec)
- [StepStatus](#stepstatus)

| Field | Description |
| --- | --- |
| `selector` _object (keys:string, values:string)_ | Cluster selector. |
| `name` _string_ | Name of the step. |
| `workspace` _string_ | Name of the workspace. |
| `command` _string_ | Bash command to run. |
| `script` _string_ | Bash script to run. |
| `env` _[EnvVar](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#envvar-v1-core) array_ | Environment variables for step. |


#### StepStatus





_Appears in:_
- [BuildManagerStatus](#buildmanagerstatus)

| Field | Description |
| --- | --- |
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `step` _[Step](#step)_ |  |


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
| `accessMode` _[PersistentVolumeAccessMode](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#persistentvolumeaccessmode-v1-core)_ | PVC access mode |


#### TLSSecretReference





_Appears in:_
- [RobotSpec](#robotspec)

| Field | Description |
| --- | --- |
| `name` _string_ | TLS secret object name. |
| `namespace` _string_ | TLS secret object namespace. |


#### Usage





_Appears in:_
- [MetricsExporterStatus](#metricsexporterstatus)

| Field | Description |
| --- | --- |
| `gpu` _[GPUUtilizationStatus](#gpuutilizationstatus)_ |  |
| `network` _[NetworkLoadStatus](#networkloadstatus)_ |  |


#### VolumeStatuses





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `varDir` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `etcDir` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `usrDir` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `optDir` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `workspaceDir` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |


#### Workspace



Workspace description. Each robot should contain at least one workspace. A workspace should contain at least one repository in it.

_Appears in:_
- [WorkspaceManagerSpec](#workspacemanagerspec)

| Field | Description |
| --- | --- |
| `name` _string_ | Name of workspace. If a workspace's name is `my_ws`, it's absolute path is `/home/workspaces/my_ws`. |
| `distro` _[ROSDistro](#rosdistro)_ |  |
| `repositories` _object (keys:string, values:[Repository](#repository))_ | Repositories to clone inside workspace's `src` directory. |


#### WorkspaceManagerInstanceStatus





_Appears in:_
- [RobotStatus](#robotstatus)

| Field | Description |
| --- | --- |
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ |  |
| `status` _[WorkspaceManagerStatus](#workspacemanagerstatus)_ |  |


#### WorkspaceManagerPhase

_Underlying type:_ `string`



_Appears in:_
- [WorkspaceManagerStatus](#workspacemanagerstatus)



