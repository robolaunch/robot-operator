# API Reference


## robot.roboscale.io/v1alpha2

Package v1alpha2 contains API Schema definitions for the robot v1alpha2 API group

### Resource Types
- [ROS2Workload](#ros2workload)
- [CodeEditor](#codeeditor)
- [EdgeProxy](#edgeproxy)
- [ROS2Bridge](#ros2bridge)



#### ROS2Workload



ROS2Workload is the Schema for the ros2workloads API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha2`
| `kind` _string_ | `ROS2Workload`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[ROS2WorkloadSpec](#ros2workloadspec)_ | Specification of the desired behavior of the ROS2Workload. |
| `status` _[ROS2WorkloadStatus](#ros2workloadstatus)_ | Most recently observed status of the ROS2Workload. |


#### ROS2WorkloadSpec



ROS2WorkloadSpec defines the desired state of ROS2Workload.

_Appears in:_
- [ROS2Workload](#ros2workload)

| Field | Description |
| --- | --- |
| `discoveryServerTemplate` _[DiscoveryServerSpec](#discoveryserverspec)_ | Discovery server configurational parameters. |
| `ros2BridgeTemplate` _[ROS2BridgeSpec](#ros2bridgespec)_ | ROS 2 Bridge configurational parameters. |
| `volumeClaimTemplates` _[PersistentVolumeClaimTemplate](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#persistentvolumeclaimtemplate-v1-core) array_ | Volume templates for ROS 2 workload. For each volume template, operator will create a PersistentVolumeClaim that can be mounted to the ROS 2 workload. |
| `launchContainers` _[LaunchContainer](#launchcontainer) array_ | Configurational parameters for containers that will be encapsulated within the ROS 2 workload StatefulSet. |


#### ROS2WorkloadStatus



ROS2WorkloadStatus defines the observed state of ROS2Workload.

_Appears in:_
- [ROS2Workload](#ros2workload)

| Field | Description |
| --- | --- |
| `phase` _ROS2WorkloadPhase_ | Phase of ROS2Workload. It sums the general status of ROS 2 workload(s). |
| `discoveryServerStatus` _[DiscoveryServerInstanceStatus](#discoveryserverinstancestatus)_ | Discovery server instance status. |
| `ros2BridgeStatus` _[ROS2BridgeInstanceStatus](#ros2bridgeinstancestatus)_ | ROS 2 Bridge instance status. |
| `pvcStatuses` _[OwnedPVCStatus](#ownedpvcstatus) array_ | Statuses of owned PersistentVolumeClaims. |
| `statefulSetStatuses` _[OwnedStatefulSetStatus](#ownedstatefulsetstatus) array_ | Status of owned StatefulSet and containers. |


#### CodeEditor



CodeEditor is the Schema for the codeeditors API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha2`
| `kind` _string_ | `CodeEditor`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[CodeEditorSpec](#codeeditorspec)_ | Specification of the desired behavior of the CodeEditor. |
| `status` _[CodeEditorStatus](#codeeditorstatus)_ | Most recently observed status of the CodeEditor. |


#### CodeEditorSpec



CodeEditorSpec defines the desired state of CodeEditor.

_Appears in:_
- [CodeEditor](#codeeditor)

| Field | Description |
| --- | --- |
| `version` _string_ | App version of the code editor. |
| `remote` _boolean_ | If `true`, code editor will be consumed remotely. |
| `container` _[CodeEditorContainer](#codeeditorcontainer)_ | Configurational parameters for code editor container. |
| `port` _integer_ | Port that code editor will use inside the container. |
| `serviceType` _[ServiceType](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#servicetype-v1-core)_ | Service type of CodeEditor. `ClusterIP` and `NodePort` is supported. |
| `ingress` _boolean_ | CodeEditor will create an Ingress resource if `true`. |
| `tlsSecretName` _string_ | Name of the TLS secret for ingress resource. |
| `volumeClaimTemplates` _[PersistentVolumeClaimTemplate](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#persistentvolumeclaimtemplate-v1-core) array_ | Volume templates for ROS 2 workload. For each volume template, operator will create a PersistentVolumeClaim that can be mounted to the ROS 2 workload. |
| `externalVolumes` _[Volume](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#volume-v1-core) array_ | External volumes. |


#### CodeEditorStatus



CodeEditorStatus defines the observed state of CodeEditor.

_Appears in:_
- [CodeEditor](#codeeditor)

| Field | Description |
| --- | --- |
| `phase` _CodeEditorPhase_ | Phase of CodeEditor. It sums the general status of code editor. |
| `pvcStatuses` _[OwnedPVCStatus](#ownedpvcstatus) array_ | Statuses of owned PersistentVolumeClaims. |
| `externalVolumeStatuses` _[ExternalVolumeStatus](#externalvolumestatus) array_ | Statuses of external volumes. |
| `deploymentStatus` _[OwnedDeploymentStatus](#owneddeploymentstatus)_ | Status of code editor deployment. |
| `serviceStatus` _[OwnedServiceStatus](#ownedservicestatus)_ | Status of code editor service. |
| `ingressStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ | Status of CodeEditor Ingress. |
| `workloadUpdateNeeded` _boolean_ | Field to indicate if the workload should be restarted. |


#### EdgeProxy



EdgeProxy is the Schema for the edgeproxies API.



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha2`
| `kind` _string_ | `EdgeProxy`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[EdgeProxySpec](#edgeproxyspec)_ |  |
| `status` _[EdgeProxyStatus](#edgeproxystatus)_ |  |






#### ROS2Bridge



ROS2Bridge is the Schema for the ros2bridges API



| Field | Description |
| --- | --- |
| `apiVersion` _string_ | `robot.roboscale.io/v1alpha2`
| `kind` _string_ | `ROS2Bridge`
| `metadata` _[ObjectMeta](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectmeta-v1-meta)_ | Refer to Kubernetes API documentation for fields of `metadata`. |
| `spec` _[ROS2BridgeSpec](#ros2bridgespec)_ | Specification of the desired behavior of the ROS2Bridge. |
| `status` _[ROS2BridgeStatus](#ros2bridgestatus)_ | Most recently observed status of the ROS2Bridge. |


#### ROS2BridgeSpec



ROS2BridgeSpec defines the desired state of ROS2Bridge.

_Appears in:_
- [ROS2Bridge](#ros2bridge)
- [ROS2WorkloadSpec](#ros2workloadspec)

| Field | Description |
| --- | --- |
| `distro` _ROSDistro_ | Configurational parameters for ROS 2 bridge. |
| `discoveryServerRef` _[ObjectReference](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectreference-v1-core)_ | Object reference to DiscoveryServer. |
| `serviceType` _[ServiceType](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#servicetype-v1-core)_ | Service type of ROS2Bridge. `ClusterIP` and `NodePort` is supported. |
| `ingress` _boolean_ | ROS2Bridge will create an Ingress resource if `true`. |
| `tlsSecretName` _string_ | Name of the TLS secret. |


#### ROS2BridgeStatus



ROS2BridgeStatus defines the observed state of ROS2Bridge.

_Appears in:_
- [ROS2Bridge](#ros2bridge)
- [ROS2BridgeInstanceStatus](#ros2bridgeinstancestatus)

| Field | Description |
| --- | --- |
| `phase` _ROS2BridgePhase_ | Phase of ROS2Bridge. |
| `connectionInfo` _[ConnectionInfo](#connectioninfo)_ | Connection info obtained from DiscoveryServer. |
| `podStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ | Status of ROS2Bridge pod. |
| `serviceStatus` _[OwnedServiceStatus](#ownedservicestatus)_ | Status of ROS2Bridge service. |
| `ingressStatus` _[OwnedResourceStatus](#ownedresourcestatus)_ | Status of ROS2Bridge Ingress. |


#### CodeEditorContainer





_Appears in:_
- [CodeEditorSpec](#codeeditorspec)

| Field | Description |
| --- | --- |
| `securityContext` _[SecurityContext](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#securitycontext-v1-core)_ | Security context of the code editor container. |
| `volumeMounts` _[VolumeMount](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#volumemount-v1-core) array_ | Mounted volumes of the code editor container. |


#### ExternalVolumeStatus





_Appears in:_
- [CodeEditorStatus](#codeeditorstatus)

| Field | Description |
| --- | --- |
| `name` _string_ | Name of the external volume. |
| `exists` _boolean_ | Indicates if the volume exists. |


#### LaunchContainer





_Appears in:_
- [ROS2WorkloadSpec](#ros2workloadspec)

| Field | Description |
| --- | --- |
| `replicas` _integer_ | Replica number of the stateful set. |
| `container` _[Container](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#container-v1-core)_ | Single container configuration for stateful set. |


#### OwnedDeploymentStatus





_Appears in:_
- [CodeEditorStatus](#codeeditorstatus)

| Field | Description |
| --- | --- |
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ | Generic status for any owned resource. |
| `status` _[DeploymentStatus](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#deploymentstatus-v1-apps)_ | Status of the Deployment. |
| `containerStatuses` _[ContainerStatus](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#containerstatus-v1-core) array_ | Container statuses. |


#### OwnedPVCStatus





_Appears in:_
- [CodeEditorStatus](#codeeditorstatus)
- [ROS2WorkloadStatus](#ros2workloadstatus)

| Field | Description |
| --- | --- |
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ | Generic status for any owned resource. |
| `status` _[PersistentVolumeClaimStatus](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#persistentvolumeclaimstatus-v1-core)_ | Status of the ROS2Bridge instance. |


#### OwnedResourceStatus



Generic status for any owned resource.

_Appears in:_
- [CodeEditorStatus](#codeeditorstatus)
- [OwnedDeploymentStatus](#owneddeploymentstatus)
- [OwnedPVCStatus](#ownedpvcstatus)
- [OwnedServiceStatus](#ownedservicestatus)
- [OwnedStatefulSetStatus](#ownedstatefulsetstatus)
- [ROS2BridgeInstanceStatus](#ros2bridgeinstancestatus)

| Field | Description |
| --- | --- |
| `created` _boolean_ | Shows if the owned resource is created. |
| `reference` _[ObjectReference](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#objectreference-v1-core)_ | Reference to the owned resource. |
| `phase` _string_ | Phase of the owned resource. |


#### OwnedServiceStatus





_Appears in:_
- [CodeEditorStatus](#codeeditorstatus)

| Field | Description |
| --- | --- |
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ | Generic status for any owned resource. |
| `urls` _object (keys:string, values:string)_ | Connection URL. |


#### OwnedStatefulSetStatus





_Appears in:_
- [ROS2WorkloadStatus](#ros2workloadstatus)

| Field | Description |
| --- | --- |
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ | Generic status for any owned resource. |
| `status` _[StatefulSetStatus](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#statefulsetstatus-v1-apps)_ | Status of the StatefulSet. |
| `containerStatuses` _[ContainerStatus](https://kubernetes.io/docs/reference/generated/kubernetes-api/v1.22/#containerstatus-v1-core) array_ | Container statuses. |


#### ROS2BridgeInstanceStatus





_Appears in:_
- [ROS2WorkloadStatus](#ros2workloadstatus)

| Field | Description |
| --- | --- |
| `resource` _[OwnedResourceStatus](#ownedresourcestatus)_ | Generic status for any owned resource. |
| `status` _[ROS2BridgeStatus](#ros2bridgestatus)_ | Status of the ROS2Bridge instance. |
| `connection` _string_ | Address of the robot service that can be reached from outside. |


