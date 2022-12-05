package v1alpha1

import (
	"github.com/robolaunch/robot-operator/internal"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

// ROS distro selection. Allowed distros are Foxy and Galactic. It is aimed to support Humble, Melodic and Noetic in further versions.
// +kubebuilder:validation:Enum=foxy;galactic
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

// RobotSpec defines the desired state of Robot
type RobotSpec struct {
	// ROS distro to be used.
	// +kubebuilder:validation:Required
	Distro ROSDistro `json:"distro"`
	// Resource limitations of robot containers.
	Storage Storage `json:"storage,omitempty"`
}

type VolumeStatus struct {
	Var       bool `json:"var,omitempty"`
	Etc       bool `json:"etc,omitempty"`
	Usr       bool `json:"usr,omitempty"`
	Opt       bool `json:"opt,omitempty"`
	Display   bool `json:"display,omitempty"`
	Workspace bool `json:"workspace,omitempty"`
}

type RobotPhase string

const (
	RobotPhaseCreatingEnvironment RobotPhase = "CreatingEnvironment"
	RobotPhaseConfiguringVolumes  RobotPhase = "ConfiguringEnvironment"
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
	VolumeStatus VolumeStatus `json:"volumeStatus,omitempty"`
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