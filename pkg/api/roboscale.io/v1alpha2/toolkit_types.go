/*
Copyright 2022.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

package v1alpha2

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func init() {
	SchemeBuilder.Register(&ROS2Bridge{}, &ROS2BridgeList{})
	SchemeBuilder.Register(&CodeEditor{}, &CodeEditorList{})
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// ROS2Bridge is the Schema for the ros2bridges API
type ROS2Bridge struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the ROS2Bridge.
	Spec ROS2BridgeSpec `json:"spec,omitempty"`
	// Most recently observed status of the ROS2Bridge.
	Status ROS2BridgeStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// ROS2BridgeList contains a list of ROS2Bridge
type ROS2BridgeList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []ROS2Bridge `json:"items"`
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// CodeEditor is the Schema for the codeeditors API
type CodeEditor struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the CodeEditor.
	Spec CodeEditorSpec `json:"spec,omitempty"`
	// Most recently observed status of the CodeEditor.
	Status CodeEditorStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// CodeEditorList contains a list of CodeEditor
type CodeEditorList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []CodeEditor `json:"items"`
}

// ********************************
// ROS2Bridge types
// ********************************

// ROS2BridgeSpec defines the desired state of ROS2Bridge.
type ROS2BridgeSpec struct {
	// Configurational parameters for ROS 2 bridge.
	// +kubebuilder:validation:Required
	Distro robotv1alpha1.ROSDistro `json:"distro"`
	// Object reference to DiscoveryServer.
	DiscoveryServerReference corev1.ObjectReference `json:"discoveryServerRef,omitempty"`
	// Service type of ROS2Bridge. `ClusterIP` and `NodePort` is supported.
	// +kubebuilder:validation:Enum=ClusterIP;NodePort
	// +kubebuilder:default=ClusterIP
	ServiceType corev1.ServiceType `json:"serviceType,omitempty"`
	// ROS2Bridge will create an Ingress resource if `true`.
	Ingress bool `json:"ingress,omitempty"`
	// Name of the TLS secret.
	TLSSecretName string `json:"tlsSecretName,omitempty"`
}

// ROS2BridgeStatus defines the observed state of ROS2Bridge.
type ROS2BridgeStatus struct {
	// Phase of ROS2Bridge.
	Phase ROS2BridgePhase `json:"phase,omitempty"`
	// Connection info obtained from DiscoveryServer.
	ConnectionInfo robotv1alpha1.ConnectionInfo `json:"connectionInfo,omitempty"`
	// Status of ROS2Bridge pod.
	PodStatus robotv1alpha1.OwnedResourceStatus `json:"podStatus,omitempty"`
	// Status of ROS2Bridge service.
	ServiceStatus robotv1alpha1.OwnedServiceStatus `json:"serviceStatus,omitempty"`
	// Status of ROS2Bridge Ingress.
	IngressStatus robotv1alpha1.OwnedResourceStatus `json:"ingressStatus,omitempty"`
}

// ********************************
// CodeEditor types
// ********************************

type ExternalVolumeStatus struct {
	// Name of the external volume.
	Name string `json:"name,omitempty"`
	// Indicates if the volume exists.
	Exists bool `json:"exists,omitempty"`
}

type CodeEditorContainer struct {
	// Security context of the code editor container.
	SecurityContext corev1.SecurityContext `json:"securityContext,omitempty"`
	// Mounted volumes of the code editor container.
	VolumeMounts []corev1.VolumeMount `json:"volumeMounts,omitempty"`
}

// CodeEditorSpec defines the desired state of CodeEditor.
type CodeEditorSpec struct {
	// If `true`, code editor will be consumed remotely.
	Remote bool `json:"remote,omitempty"`
	// Configurational parameters for code editor container.
	Container CodeEditorContainer `json:"container,omitempty"`
	// Port that code editor will use inside the container.
	// +kubebuilder:default=9000
	Port int32 `json:"port"`
	// Volume templates for ROS 2 workload.
	// For each volume template, operator will create a PersistentVolumeClaim
	// that can be mounted to the ROS 2 workload.
	VolumeClaimTemplates []corev1.PersistentVolumeClaimTemplate `json:"volumeClaimTemplates,omitempty"`
	// External volumes.
	ExternalVolumes []corev1.Volume `json:"externalVolumes,omitempty"`
}

// CodeEditorStatus defines the observed state of CodeEditor.
type CodeEditorStatus struct {
	// Phase of CodeEditor. It sums the general status of code editor.
	Phase CodeEditorPhase `json:"phase,omitempty"`
	// Statuses of owned PersistentVolumeClaims.
	PVCStatuses []OwnedPVCStatus `json:"pvcStatuses,omitempty"`
	// Statuses of external volumes.
	ExternalVolumeStatuses []ExternalVolumeStatus `json:"externalVolumeStatuses,omitempty"`
	// Status of code editor deployment.
	DeploymentStatus OwnedDeploymentStatus `json:"deploymentStatus,omitempty"`
}
