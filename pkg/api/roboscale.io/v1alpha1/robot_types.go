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
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status
//+kubebuilder:printcolumn:name="Distributions",type=string,JSONPath=`.spec.distributions`
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

// DiscoveryServer is the Schema for the discoveryservers API
type DiscoveryServer struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   DiscoveryServerSpec   `json:"spec,omitempty"`
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

// ROSBridge is the Schema for the rosbridges API
type ROSBridge struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   ROSBridgeSpec   `json:"spec,omitempty"`
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

// RobotArtifact is the Schema for the robotartifacts API
type RobotArtifact struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Template RobotSpec `json:"template,omitempty"`
}

//+kubebuilder:object:root=true

// RobotArtifactList contains a list of RobotArtifact
type RobotArtifactList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []RobotArtifact `json:"items"`
}

// ********************************
// Robot types
// ********************************

// ROS distro selection. Allowed distros are Foxy and Galactic. It is aimed to support Humble, Melodic and Noetic in further versions.
// +kubebuilder:validation:Enum=foxy;galactic;noetic;melodic;humble
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

// +kubebuilder:validation:Enum=rmw_fastrtps_cpp
type RMWImplementation string

const (
	// Cyclone DDS
	RMWImplementationCycloneDDS RMWImplementation = "rmw_cyclonedds_cpp"
	// FastRTPS
	RMWImplementationFastRTPS RMWImplementation = "rmw_fastrtps_cpp"
	// Connext
	RMWImplementationConnext RMWImplementation = "rmw_connext_cpp"
	// Gurum DDS
	RMWImplementationGurumDDS RMWImplementation = "rmw_gurumdds_cpp"
)

// Storage class configuration for a volume type.
type StorageClassConfig struct {
	// Storage class name
	Name string `json:"name,omitempty"`
	// PVC access mode
	AccessMode corev1.PersistentVolumeAccessMode `json:"accessMode,omitempty"`
}

// Robot's resource limitations.
type Storage struct {
	// Specifies how much storage will be allocated in total.
	// +kubebuilder:default=10000
	Amount int `json:"amount,omitempty"`
	// Storage class selection for robot's volumes.
	StorageClassConfig StorageClassConfig `json:"storageClassConfig,omitempty"`
}

type TLSSecretReference struct {
	// TLS secret object name.
	// +kubebuilder:validation:Required
	Name string `json:"name"`
	// TLS secret object namespace.
	// +kubebuilder:validation:Required
	Namespace string `json:"namespace"`
}

type RootDNSConfig struct {
	// DNS host.
	// +kubebuilder:validation:Required
	Host string `json:"host"`
}

// RobotSpec defines the desired state of Robot.
type RobotSpec struct {
	// ROS 2 distributions to be used. You can select multiple distributions if they are supported in the same underlying OS.
	// (eg. `foxy` and `galactic` are supported in Ubuntu Focal, so they can be used together but both cannot be used with `humble`)
	// +kubebuilder:validation:Required
	// +kubebuilder:validation:MinItems=1
	// +kubebuilder:validation:MaxItems=2
	Distributions []ROSDistro `json:"distributions"`
	// RMW implementation selection. Robot operator currently supports only FastRTPS. See https://docs.ros.org/en/foxy/How-To-Guides/Working-with-multiple-RMW-implementations.html.
	// +kubebuilder:validation:Required
	// +kubebuilder:default=rmw_fastrtps_cpp
	RMWImplementation RMWImplementation `json:"rmwImplementation"`
	// Total storage amount to persist via Robot. Unit of measurement is MB. (eg. `10240` corresponds 10 GB)
	// This amount is being shared between different components.
	Storage Storage `json:"storage,omitempty"`
	// Discovery server configurational parameters.
	DiscoveryServerTemplate DiscoveryServerSpec `json:"discoveryServerTemplate,omitempty"`
	// ROS bridge configurational parameters.
	ROSBridgeTemplate ROSBridgeSpec `json:"rosBridgeTemplate,omitempty"`
	// Robot development suite template
	RobotDevSuiteTemplate RobotDevSuiteSpec `json:"robotDevSuiteTemplate,omitempty"`
	// Workspace manager template to configure ROS 2 workspaces.
	WorkspaceManagerTemplate WorkspaceManagerSpec `json:"workspaceManagerTemplate,omitempty"`
	// [*alpha*] Build manager template for initial configuration.
	BuildManagerTemplate BuildManagerSpec `json:"buildManagerTemplate,omitempty"`
	// [*alpha*] Launch manager template for initial configuration.
	LaunchManagerTemplates []LaunchManagerSpec `json:"launchManagerTemplates,omitempty"`
	// [*alpha*] Switch to development mode if `true`.
	Development bool `json:"development,omitempty"`
	// [*alpha*] Root DNS configuration.
	RootDNSConfig RootDNSConfig `json:"rootDNSConfig,omitempty"`
	// [*alpha*] TLS secret reference.
	TLSSecretReference TLSSecretReference `json:"tlsSecretRef,omitempty"`
}

type VolumeStatuses struct {
	Var       OwnedResourceStatus `json:"varDir,omitempty"`
	Etc       OwnedResourceStatus `json:"etcDir,omitempty"`
	Usr       OwnedResourceStatus `json:"usrDir,omitempty"`
	Opt       OwnedResourceStatus `json:"optDir,omitempty"`
	Workspace OwnedResourceStatus `json:"workspaceDir,omitempty"`
}

type JobPhase string

const (
	JobActive    JobPhase = "Active"
	JobSucceeded JobPhase = "Succeeded"
	JobFailed    JobPhase = "Failed"
)

type AttachedBuildObject struct {
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	Status    BuildManagerStatus     `json:"status,omitempty"`
}

type AttachedLaunchObject struct {
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	Status    LaunchManagerStatus    `json:"status,omitempty"`
}

type AttachedDevObject struct {
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	Status    RobotDevSuiteStatus    `json:"status,omitempty"`
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

type DiscoveryServerInstanceType string

const (
	DiscoveryServerInstanceTypeServer DiscoveryServerInstanceType = "Server"
	DiscoveryServerInstanceTypeClient DiscoveryServerInstanceType = "Client"
)

// DiscoveryServerSpec defines the desired state of DiscoveryServer
type DiscoveryServerSpec struct {
	Type      DiscoveryServerInstanceType `json:"type,omitempty"`
	Reference corev1.ObjectReference      `json:"reference,omitempty"`
	Cluster   string                      `json:"cluster,omitempty"`
	Hostname  string                      `json:"hostname,omitempty"`
	Subdomain string                      `json:"subdomain,omitempty"`
	Image     string                      `json:"image,omitempty"`
	Args      []string                    `json:"args,omitempty"`
}

type ConnectionInfo struct {
	IP            string `json:"ip,omitempty"`
	ConfigMapName string `json:"configMapName,omitempty"`
}

// DiscoveryServerStatus defines the observed state of DiscoveryServer
type DiscoveryServerStatus struct {
	Phase               DiscoveryServerPhase `json:"phase,omitempty"`
	ServiceStatus       OwnedResourceStatus  `json:"serviceStatus,omitempty"`
	ServiceExportStatus OwnedResourceStatus  `json:"serviceExportStatus,omitempty"`
	PodStatus           OwnedPodStatus       `json:"podStatus,omitempty"`
	ConfigMapStatus     OwnedResourceStatus  `json:"configMapStatus,omitempty"`
	ConnectionInfo      ConnectionInfo       `json:"connectionInfo,omitempty"`
}

// ********************************
// ROSBridge types
// ********************************

type BridgeDistro struct {
	Enabled bool      `json:"enabled,omitempty"`
	Distro  ROSDistro `json:"distro,omitempty"`
}

// ROSBridgeSpec defines the desired state of ROSBridge
type ROSBridgeSpec struct {
	ROS   BridgeDistro `json:"ros,omitempty"`
	ROS2  BridgeDistro `json:"ros2,omitempty"`
	Image string       `json:"image,omitempty"`
}

// ROSBridgeStatus defines the observed state of ROSBridge
type ROSBridgeStatus struct {
	Phase         BridgePhase         `json:"phase,omitempty"`
	PodStatus     OwnedResourceStatus `json:"podStatus,omitempty"`
	ServiceStatus OwnedResourceStatus `json:"serviceStatus,omitempty"`
}

// ********************************
// RobotArtifact types
// ********************************
