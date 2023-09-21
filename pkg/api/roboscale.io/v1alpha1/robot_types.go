package v1alpha1

import (
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func init() {
	SchemeBuilder.Register(&Robot{}, &RobotList{})
	SchemeBuilder.Register(&ROSBridge{}, &ROSBridgeList{})
	SchemeBuilder.Register(&DiscoveryServer{}, &DiscoveryServerList{})
	SchemeBuilder.Register(&RobotArtifact{}, &RobotArtifactList{})
	SchemeBuilder.Register(&RelayServer{}, &RelayServerList{})
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status
//+kubebuilder:printcolumn:name="Distributions",type=string,JSONPath=`.spec.robot.distributions`
//+kubebuilder:printcolumn:name="Application",type=string,JSONPath=`.spec.environment.application.name`
//+kubebuilder:printcolumn:name="Version",type=string,JSONPath=`.spec.environment.application.version`
//+kubebuilder:printcolumn:name="Ubuntu",type=string,JSONPath=`.spec.environment.devspace.ubuntuDistro`
//+kubebuilder:printcolumn:name="DevSpace",type=string,JSONPath=`.spec.environment.devspace.version`
//+kubebuilder:printcolumn:name="Phase",type=string,JSONPath=`.status.phase`

// Robot is the custom resource that contains ROS 2 components (Workloads, Cloud VDI, Cloud IDE, ROS Bridge, Configurational Resources), robolaunch Robot instances can be decomposed and distributed to both cloud instances and physical instances using federation.
type Robot struct {
	metav1.TypeMeta `json:",inline"`
	// Standard object's metadata.
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the Robot.
	Spec RobotSpec `json:"spec,omitempty"`
	// Most recently observed status of the Robot.
	Status RobotStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// RobotList contains a list of Robot
type RobotList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []Robot `json:"items"`
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status
//+kubebuilder:printcolumn:name="Type",type=string,JSONPath=`.spec.type`
//+kubebuilder:printcolumn:name="Domain ID",type=string,JSONPath=`.spec.domainID`
//+kubebuilder:printcolumn:name="Hostname",type=string,JSONPath=`.spec.hostname`
//+kubebuilder:printcolumn:name="Subdomain",type=string,JSONPath=`.spec.subdomain`
//+kubebuilder:printcolumn:name="IP",type=string,JSONPath=`.status.connectionInfo.ip`
//+kubebuilder:printcolumn:name="URI",type=string,JSONPath=`.status.connectionInfo.uri`
//+kubebuilder:printcolumn:name="Phase",type=string,JSONPath=`.status.phase`

// DiscoveryServer is a custom resource that connects Robots and Fleets
// both locally and geoghraphically in DDS (UDP multicast) level.
type DiscoveryServer struct {
	metav1.TypeMeta `json:",inline"`
	// Standard object's metadata.
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the DiscoveryServer.
	Spec DiscoveryServerSpec `json:"spec,omitempty"`
	// Most recently observed status of the DiscoveryServer.
	Status DiscoveryServerStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// DiscoveryServerList contains a list of DiscoveryServer
type DiscoveryServerList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []DiscoveryServer `json:"items"`
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status
//+kubebuilder:printcolumn:name="Phase",type=string,JSONPath=`.status.phase`

// ROSBridge is a custom resource that provisions ROS/2 bridge resources and workloads.
// It could also convert ROS 2 topics to ROS topics using ROS 1 to 2 bridge.
// (see https://github.com/ros2/ros1_bridge)
type ROSBridge struct {
	metav1.TypeMeta `json:",inline"`
	// Standard object's metadata.
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the ROSBridge.
	Spec ROSBridgeSpec `json:"spec,omitempty"`
	// Most recently observed status of the ROSBridge.
	Status ROSBridgeStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// ROSBridgeList contains a list of ROSBridge
type ROSBridgeList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []ROSBridge `json:"items"`
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// RobotArtifact is a non-functional resource that holds Robot's specifications.
// It is used to define Robot templates.
type RobotArtifact struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Holds Robot's `.spec`.
	Template RobotSpec `json:"template,omitempty"`
}

//+kubebuilder:object:root=true

// RobotArtifactList contains a list of RobotArtifact
type RobotArtifactList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []RobotArtifact `json:"items"`
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status
//+kubebuilder:printcolumn:name="Instance",type=string,JSONPath=`.spec.instanceName`
//+kubebuilder:printcolumn:name="Remote Namespace",type=string,JSONPath=`.spec.remoteNamespace`
//+kubebuilder:printcolumn:name="Phase",type=string,JSONPath=`.status.phase`

// RelayServer is the Schema for the relayservers API
type RelayServer struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   RelayServerSpec   `json:"spec,omitempty"`
	Status RelayServerStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// RelayServerList contains a list of RelayServer
type RelayServerList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []RelayServer `json:"items"`
}

// ********************************
// Robot types
// ********************************

type Type string

const (
	TypeEnvironment Type = "Environment"
	TypeRobot       Type = "Robot"
)

// ROS 2 distribution selection. Currently supported distributions are Humble, Foxy, Galactic.
// +kubebuilder:validation:Enum=foxy;galactic;humble
type ROSDistro string

const (
	// ROS Melodic Morenia
	ROSDistroMelodic ROSDistro = "melodic"
	// ROS Noetic Ninjemys
	ROSDistroNoetic ROSDistro = "noetic"
	// ROS 2 Foxy Fitzroy
	ROSDistroFoxy ROSDistro = "foxy"
	// ROS 2 Galactic Geochelone
	ROSDistroGalactic ROSDistro = "galactic"
	// ROS 2 Humble Hawksbill
	ROSDistroHumble ROSDistro = "humble"
)

// RMW implementation chooses DDS vendor for ROS 2. Currently, only eProsima's FastDDS is supported.
// +kubebuilder:validation:Enum=rmw_fastrtps_cpp
type RMWImplementation string

const (
	// Cyclone DDS
	RMWImplementationCycloneDDS RMWImplementation = "rmw_cyclonedds_cpp"
	// FastDDS
	RMWImplementationFastRTPS RMWImplementation = "rmw_fastrtps_cpp"
	// Connext
	RMWImplementationConnext RMWImplementation = "rmw_connext_cpp"
	// Gurum DDS
	RMWImplementationGurumDDS RMWImplementation = "rmw_gurumdds_cpp"
)

// Storage class configuration for a volume type.
type StorageClassConfig struct {
	// Storage class name.
	Name string `json:"name,omitempty"`
	// PVC access modes. Currently, only `ReadWriteOnce` is supported.
	AccessMode corev1.PersistentVolumeAccessMode `json:"accessMode,omitempty"`
}

// Robot's resource limitations.
type Storage struct {
	// Specifies how much storage will be allocated in total. Use MB as a unit of measurement. (eg. `10240` is equal to 10 GB)
	// +kubebuilder:default=10000
	Amount int `json:"amount,omitempty"`
	// Storage class selection for robot's volumes.
	StorageClassConfig StorageClassConfig `json:"storageClassConfig,omitempty"`
}

type TLSSecretReference struct {
	// [*alpha*] TLS secret object name.
	// +kubebuilder:validation:Required
	Name string `json:"name"`
	// [*alpha*] TLS secret object namespace.
	// +kubebuilder:validation:Required
	Namespace string `json:"namespace"`
}

type RootDNSConfig struct {
	// [*alpha*] Root DNS name..
	// +kubebuilder:validation:Required
	Host string `json:"host"`
}

type RobotConfig struct {
	// ROS 2 distributions to be used. You can select multiple distributions if they are supported in the same underlying OS.
	// (eg. `foxy` and `galactic` are supported in Ubuntu Focal, so they can be used together but both cannot be used with `humble`)
	// +kubebuilder:validation:Required
	// +kubebuilder:validation:MinItems=1
	// +kubebuilder:validation:MaxItems=2
	Distributions []ROSDistro `json:"distributions,omitempty"`
	// RMW implementation selection. Robot operator currently supports only FastRTPS. See https://docs.ros.org/en/foxy/How-To-Guides/Working-with-multiple-RMW-implementations.html.
	// +kubebuilder:validation:Required
	RMWImplementation RMWImplementation `json:"rmwImplementation,omitempty"`
	// ROS domain ID for robot. See https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html.
	// +kubebuilder:validation:Minimum=0
	// +kubebuilder:validation:Maximum=101
	DomainID int `json:"domainID"`
	// Discovery server configurational parameters.
	DiscoveryServerTemplate DiscoveryServerSpec `json:"discoveryServerTemplate,omitempty"`
	// ROS bridge configurational parameters.
	ROSBridgeTemplate ROSBridgeSpec `json:"rosBridgeTemplate,omitempty"`
}

type Application struct {
	// Application name.
	// +kubebuilder:validation:Required
	Name string `json:"name"`
	// Version of the application.
	// +kubebuilder:validation:Required
	Version string `json:"version"`
}

type DevSpaceImage struct {
	// Ubuntu distribution of the environment.
	// +kubebuilder:validation:Required
	UbuntuDistro string `json:"ubuntuDistro"`
	// Ubuntu desktop.
	// +kubebuilder:validation:Required
	Desktop string `json:"desktop"`
	// DevSpace image version.
	// +kubebuilder:validation:Required
	Version string `json:"version"`
}

type EnvironmentConfig struct {
	// Domain of the environment.
	// +kubebuilder:validation:Required
	Domain string `json:"domain"`
	// Application properties.
	// +kubebuilder:validation:Required
	Application Application `json:"application"`
	// DevSpace image properties.
	// +kubebuilder:validation:Required
	DevSpaceImage DevSpaceImage `json:"devspace"`
}

// RobotSpec defines the desired state of Robot.
type RobotSpec struct {
	// Determines the object type.
	// If "Environment", operator will provision an environment according to the specifications. (`.spec.environment`)
	// If "Robot", operator will provision an environment specialized for ROS 2 according to the specifications. (`.spec.robot`)
	Type Type `json:"type,omitempty"`
	// Holds robot's configuration.
	// Applied if `.spec.type` is `Robot` and must be `nil` otherwise.
	// +kubebuilder:validation:Optional
	RobotConfig RobotConfig `json:"robot,omitempty"`
	// Holds environment's configuration.
	// Applied if `.spec.type` is `Environment` and must be `nil` otherwise.
	// +kubebuilder:validation:Optional
	EnvironmentConfig EnvironmentConfig `json:"environment,omitempty"`
	// User ID of robolaunch user in image.
	// +kubebuilder:default=1000
	UID int64 `json:"uid,omitempty"`
	// Total storage amount to persist via Robot. Unit of measurement is MB. (eg. `10240` corresponds 10 GB)
	// This amount is being shared between different components.
	Storage Storage `json:"storage,omitempty"`
	// Robot development suite template
	RobotDevSuiteTemplate RobotDevSuiteSpec `json:"robotDevSuiteTemplate,omitempty"`
	// Workspace manager template to configure ROS 2 workspaces.
	WorkspaceManagerTemplate WorkspaceManagerSpec `json:"workspaceManagerTemplate,omitempty"`
	// [*alpha*] Root DNS configuration.
	RootDNSConfig RootDNSConfig `json:"rootDNSConfig,omitempty"`
	// [*alpha*] TLS secret reference.
	TLSSecretReference TLSSecretReference `json:"tlsSecretRef,omitempty"`
}

type VolumeStatuses struct {
	// Holds PVC status of the `/var` directory of underlying OS.
	Var OwnedResourceStatus `json:"varDir,omitempty"`
	// Holds PVC status of the `/etc` directory of underlying OS.
	Etc OwnedResourceStatus `json:"etcDir,omitempty"`
	// Holds PVC status of the `/usr` directory of underlying OS.
	Usr OwnedResourceStatus `json:"usrDir,omitempty"`
	// Holds PVC status of the `/opt` directory of underlying OS.
	Opt OwnedResourceStatus `json:"optDir,omitempty"`
	// Holds PVC status of the workspaces directory of underlying OS.
	Workspace OwnedResourceStatus `json:"workspaceDir,omitempty"`
}

type JobPhase string

const (
	JobActive    JobPhase = "Active"
	JobSucceeded JobPhase = "Succeeded"
	JobFailed    JobPhase = "Failed"
)

type AttachedBuildObject struct {
	// Reference to the BuildManager instance.
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	// Status of attached BuildManager.
	Status BuildManagerStatus `json:"status,omitempty"`
}

type AttachedLaunchObject struct {
	// Reference to the LaunchManager instance.
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	// Status of attached LaunchManager.
	Status LaunchManagerStatus `json:"status,omitempty"`
}

type AttachedDevObject struct {
	// Reference to the RobotDevSuite instance.
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	// Status of attached RobotDevSuite.
	Status RobotDevSuiteStatus `json:"status,omitempty"`
}

// RobotStatus defines the observed state of Robot.
type RobotStatus struct {
	// Phase of Robot. It sums the general status of Robot.
	Phase RobotPhase `json:"phase,omitempty"`
	// Main image of Robot. It is derived either from the specifications or determined directly over labels.
	Image string `json:"image,omitempty"`
	// Node that Robot uses. It is selected via tenancy labels.
	NodeName string `json:"nodeName,omitempty"`
	// Robot persists some of the directories of underlying OS inside persistent volumes.
	// This field exposes persistent volume claims that dynamically provision PVs.
	VolumeStatuses VolumeStatuses `json:"volumeStatuses,omitempty"`
	// Discovery server instance status.
	DiscoveryServerStatus DiscoveryServerInstanceStatus `json:"discoveryServerStatus,omitempty"`
	// ROS bridge instance status.
	ROSBridgeStatus ROSBridgeInstanceStatus `json:"rosBridgeStatus,omitempty"`
	// Status of loader job that configures environment.
	LoaderJobStatus OwnedResourceStatus `json:"loaderJobStatus,omitempty"`
	// Workspace manager instance status if exists.
	WorkspaceManagerStatus WorkspaceManagerInstanceStatus `json:"workspaceManagerStatus,omitempty"`
	// Robot development suite instance status.
	RobotDevSuiteStatus RobotDevSuiteInstanceStatus `json:"robotDevSuiteStatus,omitempty"`
	// Attached build object information.
	// A BuildManager can be attached with a label on it with key `robolaunch.io/target-robot`
	// and value of the target robot's name.
	// Robot sorts the BuildManagers targeted itself, and picks the last created object to process.
	AttachedBuildObject AttachedBuildObject `json:"attachedBuildObject,omitempty"`
	// Attached launch object information.
	// A LaunchManager can be attached with a label on it with key `robolaunch.io/target-robot`
	// and value of the target robot's name.
	// Multiple LaunchManager could work together if they targeted the same Robot.
	AttachedLaunchObjects []AttachedLaunchObject `json:"attachedLaunchObjects,omitempty"`
	// [*alpha*] Initial build manager creation status if exists.
	InitialBuildManagerStatus OwnedResourceStatus `json:"initialBuildManagerStatus,omitempty"`
	// [*alpha*] Initial launch manager creation status if exists.
	InitialLaunchManagerStatuses []OwnedResourceStatus `json:"initialLaunchManagerStatuses,omitempty"`
	// [*alpha*] Attached dev object information.
	AttachedDevObjects []AttachedDevObject `json:"attachedDevObjects,omitempty"`
}

// ********************************
// DiscoveryServer types
// ********************************

// Instance type can be either `Server` or `Client`.
type DiscoveryServerInstanceType string

const (
	DiscoveryServerInstanceTypeServer DiscoveryServerInstanceType = "Server"
	DiscoveryServerInstanceTypeClient DiscoveryServerInstanceType = "Client"
)

// DiscoveryServerSpec defines the desired state of DiscoveryServer.
type DiscoveryServerSpec struct {
	// ROS domain ID for robot. See https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html.
	// +kubebuilder:validation:Minimum=0
	// +kubebuilder:validation:Maximum=101
	DomainID int `json:"domainID"`
	// Instance type can be either `Server` or `Client`.
	// If `Server`, instance creates discovery server resources and workloads.
	// Other `Client` instances can connect to the `Server` instance.
	// If `Client`, instance tries to connect a `Server` instance and hold `Server` configuration in a ConfigMap.
	Type DiscoveryServerInstanceType `json:"type,omitempty"`
	// Reference to the `Server` instance.
	// It is used if `.spec.type` is `Client`.
	// Referenced object can be previously provisioned in another cluster.
	// In that case, cluster's name can be specified in `.spec.cluster` field.
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	// Cloud instance name that holds DiscoveryServer instance with `Server` type.
	// Should be empty if the type is `Server` since it takes cloud instance's name automatically.
	// Should be set if the type is `Client`.
	Cluster string `json:"cluster,omitempty"`
	// If instance type is `Server`, it can be an arbitrary value.
	// If instance type is `Client`, it should be the same with Server's hostname.
	// Used for getting Server's IP over DNS.
	Hostname string `json:"hostname,omitempty"`
	// If instance type is `Server`, it can be an arbitrary value.
	// If instance type is `Client`, it should be the same with Server's subdomain.
	// Used for getting Server's IP over DNS.
	Subdomain string `json:"subdomain,omitempty"`
}

type ConnectionInfo struct {
	// URI of the discovery server.
	// Discovery server instance tries to ping this address to see if it's reachable.
	URI string `json:"uri,omitempty"`
	// IP of the discovery server.
	// IP is being obtained from the DNS name, which is being built according to the discovery server configuration.
	IP string `json:"ip,omitempty"`
	// Name of the config map that holds discovery server configuration.
	ConfigMapName string `json:"configMapName,omitempty"`
}

// DiscoveryServerStatus defines the observed state of DiscoveryServer.
type DiscoveryServerStatus struct {
	// Phase of the DiscoveryServer.
	Phase DiscoveryServerPhase `json:"phase,omitempty"`
	// Status of the DiscoveryServer service.
	ServiceStatus OwnedResourceStatus `json:"serviceStatus,omitempty"`
	// Status of the DiscoveryServer ServiceExport.
	ServiceExportStatus OwnedResourceStatus `json:"serviceExportStatus,omitempty"`
	// Status of the DiscoveryServer pod.
	PodStatus OwnedPodStatus `json:"podStatus,omitempty"`
	// Status of the DiscoveryServer config map.
	ConfigMapStatus OwnedResourceStatus `json:"configMapStatus,omitempty"`
	// Connection information.
	ConnectionInfo ConnectionInfo `json:"connectionInfo,omitempty"`
}

// ********************************
// ROSBridge types
// ********************************

type BridgeDistro struct {
	// If `true`, resources and workloads are created by ROSBridge.
	Enabled bool `json:"enabled,omitempty"`
	// ROS distribution for bridge.
	Distro ROSDistro `json:"distro,omitempty"`
}

// ROSBridgeSpec defines the desired state of ROSBridge.
type ROSBridgeSpec struct {
	// Configurational parameters for ROS bridge.
	ROS BridgeDistro `json:"ros,omitempty"`
	// Configurational parameters for ROS 2 bridge.
	ROS2 BridgeDistro `json:"ros2,omitempty"`
	// Service type of ROSBridge. `ClusterIP` and `NodePort` is supported.
	// +kubebuilder:validation:Enum=ClusterIP;NodePort
	ServiceType corev1.ServiceType `json:"serviceType,omitempty"`
	// [*alpha*] ROSBridge will create an Ingress resource if `true`.
	Ingress bool `json:"ingress,omitempty"`
}

// ROSBridgeStatus defines the observed state of ROSBridge.
type ROSBridgeStatus struct {
	// Phase of ROSBridge.
	Phase BridgePhase `json:"phase,omitempty"`
	// Status of ROSBridge pod.
	PodStatus OwnedResourceStatus `json:"podStatus,omitempty"`
	// Status of ROSBridge service.
	ServiceStatus OwnedServiceStatus `json:"serviceStatus,omitempty"`
	// Status of ROSBridge Ingress.
	IngressStatus OwnedResourceStatus `json:"ingressStatus,omitempty"`
}

// ********************************
// RobotArtifact types
// ********************************

// ********************************
// RelayServer types
// ********************************

// RelayServerSpec defines the desired state of RelayServer.
type RelayServerSpec struct {
	// Hostname of the remote pod.
	Hostname string `json:"hostname,omitempty"`
	// Subdomain of the remote pod. It's also same with remote service's name.
	Subdomain string `json:"subdomain,omitempty"`
	// Remote instance name.
	InstanceName string `json:"instanceName,omitempty"`
	// Remote namespace.
	RemoteNamespace string `json:"remoteNamespace,omitempty"`
	// Remote port.
	RemotePort int `json:"remotePort,omitempty"`
	// [*alpha*] Root DNS configuration.
	RootDNSConfig RootDNSConfig `json:"rootDNSConfig,omitempty"`
	// [*alpha*] TLS secret reference.
	TLSSecretReference TLSSecretReference `json:"tlsSecretRef,omitempty"`
}

// RelayServerStatus defines the observed state of RelayServer.
type RelayServerStatus struct {
	// Phase of RelayServer.
	Phase RelayServerPhase `json:"phase,omitempty"`
	// Status of RelayServer pod.
	PodStatus OwnedResourceStatus `json:"podStatus,omitempty"`
	// Status of RelayServer service.
	ServiceStatus OwnedServiceStatus `json:"serviceStatus,omitempty"`
	// Status of RelayServer Ingress.
	IngressStatus OwnedResourceStatus `json:"ingressStatus,omitempty"`
}
