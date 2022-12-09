package v1alpha1

import (
	"github.com/robolaunch/robot-operator/internal"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

// ROS distro selection. Allowed distros are Foxy and Galactic. It is aimed to support Humble, Melodic and Noetic in further versions.
// +kubebuilder:validation:Enum=foxy;galactic;noetic;melodic
type ROSDistro string

const (
	// ROS Melodic Morenia
	ROSDistroMelodic ROSDistro = "melodic"
	// ROS Noetic Ninjemys
	ROSDistroNoetic ROSDistro = "noetic"
	// ROS Foxy Fitzroy
	ROSDistroFoxy ROSDistro = "foxy"
	// ROS Galactic Geochelone
	ROSDistroGalactic ROSDistro = "galactic"
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

// Repository description.
type Repository struct {
	// Name of the repository.
	// +kubebuilder:validation:Required
	Name string `json:"name"`
	// Base URL of the repository.
	// +kubebuilder:validation:Required
	URL string `json:"url"`
	// Branch of the repository to clone.
	// +kubebuilder:validation:Required
	Branch string `json:"branch"`
	// [Autofilled] Absolute path of repository
	Path string `json:"path,omitempty"`
}

// Workspace description. Each robot should contain at least one workspace. A workspace should contain at least one
// repository in it.
type Workspace struct {
	// Name of workspace. If a workspace's name is `my_ws`, it's absolute path is `/home/workspaces/my_ws`.
	// +kubebuilder:validation:Required
	Name string `json:"name"`
	// Repositories to clone inside workspace's `src` directory.
	// +kubebuilder:validation:MinItems=1
	Repositories []Repository `json:"repositories"`
}

// RobotSpec defines the desired state of Robot
type RobotSpec struct {
	// ROS distro to be used.
	// +kubebuilder:validation:Required
	Distro ROSDistro `json:"distro"`
	// Resource limitations of robot containers.
	Storage Storage `json:"storage,omitempty"`
	// Discovery server template
	DiscoveryServerTemplate DiscoveryServerSpec `json:"discoveryServerTemplate,omitempty"`
	// ROS bridge template
	ROSBridgeTemplate ROSBridgeSpec `json:"rosBridgeTemplate,omitempty"`
	// Global path of workspaces. It's fixed to `/home/workspaces` path.
	WorkspacesPath string `json:"workspacesPath,omitempty"`
	// Workspace definitions of robot.
	// +kubebuilder:validation:MinItems=1
	Workspaces []Workspace `json:"workspaces,omitempty"`
}

type VolumeStatus struct {
	Created                   bool   `json:"created,omitempty"`
	PersistentVolumeClaimName string `json:"persistentVolumeClaimName,omitempty"`
}

type VolumeStatuses struct {
	Var       VolumeStatus `json:"var,omitempty"`
	Etc       VolumeStatus `json:"etc,omitempty"`
	Usr       VolumeStatus `json:"usr,omitempty"`
	Opt       VolumeStatus `json:"opt,omitempty"`
	Display   VolumeStatus `json:"display,omitempty"`
	Workspace VolumeStatus `json:"workspace,omitempty"`
}

type DiscoveryServerInstanceStatus struct {
	Created bool                  `json:"created,omitempty"`
	Status  DiscoveryServerStatus `json:"status,omitempty"`
}

type ROSBridgeInstanceStatus struct {
	Created bool            `json:"created,omitempty"`
	Status  ROSBridgeStatus `json:"status,omitempty"`
}

type JobPhase string

const (
	JobActive    JobPhase = "Active"
	JobSucceeded JobPhase = "Succeeded"
	JobFailed    JobPhase = "Failed"
)

type LoaderJobStatus struct {
	Created bool     `json:"created,omitempty"`
	Phase   JobPhase `json:"phase,omitempty"`
}

type RobotPhase string

const (
	RobotPhaseCreatingEnvironment     RobotPhase = "CreatingEnvironment"
	RobotPhaseCreatingDiscoveryServer RobotPhase = "CreatingDiscoveryServer"
	RobotPhaseConfiguringWorkspaces   RobotPhase = "ConfiguringWorkspaces"
	RobotPhaseCreatingBridge          RobotPhase = "CreatingBridge"
	RobotPhaseReady                   RobotPhase = "Ready"
	RobotPhaseDeletingBridge          RobotPhase = "DeletingBridge"
	RobotPhaseDeletingDiscoveryServer RobotPhase = "DeletingDiscoveryServer"
	RobotPhaseDeletingLoaderJob       RobotPhase = "DeletingLoaderJob"
	RobotPhaseDeletingVolumes         RobotPhase = "DeletingVolumes"

	RobotPhaseFailed RobotPhase = "Failed"
)

// RobotStatus defines the observed state of Robot
type RobotStatus struct {
	// Phase of robot
	Phase RobotPhase `json:"phase,omitempty"`
	// Image of robot
	Image string `json:"image,omitempty"`
	// Node name
	NodeName string `json:"nodeName,omitempty"`
	// Volume status
	VolumeStatuses VolumeStatuses `json:"volumeStatuses,omitempty"`
	// Discovery server instance status
	DiscoveryServerStatus DiscoveryServerInstanceStatus `json:"discoveryServerStatus,omitempty"`
	// ROS bridge instance status
	ROSBridgeStatus ROSBridgeInstanceStatus `json:"rosBridgeStatus,omitempty"`
	// Loader job status that configures environment
	LoaderJobStatus LoaderJobStatus `json:"loaderJobStatus,omitempty"`
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// Robot is the Schema for the robots API
type Robot struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   RobotSpec   `json:"spec,omitempty"`
	Status RobotStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// RobotList contains a list of Robot
type RobotList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []Robot `json:"items"`
}

func init() {
	SchemeBuilder.Register(&Robot{}, &RobotList{})
}

func (robot *Robot) GetPVCVarMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_VAR_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetPVCOptMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_OPT_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetPVCUsrMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_USR_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetPVCEtcMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_ETC_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetPVCDisplayMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_DISPLAY_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetPVCWorkspaceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.PVC_WORKSPACE_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetDiscoveryServerMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.DISCOVERY_SERVER_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetLoaderJobMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.JOB_LOADER_POSTFIX,
		Namespace: robot.Namespace,
	}
}

func (robot *Robot) GetROSBridgeMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      robot.Name + internal.ROS_BRIDGE_POSTFIX,
		Namespace: robot.Namespace,
	}
}
