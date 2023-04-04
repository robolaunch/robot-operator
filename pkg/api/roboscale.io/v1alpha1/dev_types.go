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

package v1alpha1

import (
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func init() {
	SchemeBuilder.Register(&RobotDevSuite{}, &RobotDevSuiteList{})
	SchemeBuilder.Register(&RobotIDE{}, &RobotIDEList{})
	SchemeBuilder.Register(&RobotVDI{}, &RobotVDIList{})
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// RobotDevSuite is the Schema for the robotdevsuites API
type RobotDevSuite struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   RobotDevSuiteSpec   `json:"spec,omitempty"`
	Status RobotDevSuiteStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// RobotDevSuiteList contains a list of RobotDevSuite
type RobotDevSuiteList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []RobotDevSuite `json:"items"`
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// RobotIDE is the Schema for the robotides API
type RobotIDE struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   RobotIDESpec   `json:"spec,omitempty"`
	Status RobotIDEStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// RobotIDEList contains a list of RobotIDE
type RobotIDEList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []RobotIDE `json:"items"`
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// RobotVDI is the Schema for the robotvdis API
type RobotVDI struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   RobotVDISpec   `json:"spec,omitempty"`
	Status RobotVDIStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// RobotVDIList contains a list of RobotVDI
type RobotVDIList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []RobotVDI `json:"items"`
}

// ********************************
// RobotDevSuite types
// ********************************

// RobotDevSuiteSpec defines the desired state of RobotDevSuite
type RobotDevSuiteSpec struct {
	VDIEnabled       bool         `json:"vdiEnabled,omitempty"`
	RobotVDITemplate RobotVDISpec `json:"robotVDITemplate,omitempty"`
	IDEEnabled       bool         `json:"ideEnabled,omitempty"`
	RobotIDETemplate RobotIDESpec `json:"robotIDETemplate,omitempty"`
}

// RobotDevSuiteStatus defines the observed state of RobotDevSuite
type RobotDevSuiteStatus struct {
	Phase          RobotDevSuitePhase  `json:"phase,omitempty"`
	Active         bool                `json:"active,omitempty"`
	RobotVDIStatus OwnedResourceStatus `json:"robotVDIStatus,omitempty"`
	RobotIDEStatus OwnedResourceStatus `json:"robotIDEStatus,omitempty"`
}

// ********************************
// RobotIDE types
// ********************************

// RobotIDESpec defines the desired state of RobotIDE
type RobotIDESpec struct {
	Resources Resources `json:"resources,omitempty"`
	// ServiceType
	// +kubebuilder:validation:Enum=ClusterIP;NodePort
	// +kubebuilder:default="NodePort"
	ServiceType corev1.ServiceType `json:"serviceType,omitempty"`
	Ingress     bool               `json:"ingress,omitempty"`
	Privileged  bool               `json:"privileged,omitempty"`
	// Display configuration.
	Display bool `json:"display,omitempty"`
}

// RobotIDEStatus defines the observed state of RobotIDE
type RobotIDEStatus struct {
	Phase         RobotIDEPhase       `json:"phase,omitempty"`
	PodStatus     OwnedPodStatus      `json:"podStatus,omitempty"`
	ServiceStatus OwnedResourceStatus `json:"serviceStatus,omitempty"`
	IngressStatus OwnedResourceStatus `json:"ingressStatus,omitempty"`
}

// ********************************
// RobotVDI types
// ********************************

// VDI resource limits.
type Resources struct {
	GPUCore int `json:"gpuCore,omitempty"`
	// +kubebuilder:validation:Pattern=`^([0-9])+(m)$`
	CPU string `json:"cpu,omitempty"`
	// +kubebuilder:validation:Pattern=`^([0-9])+(Mi|Gi)$`
	Memory string `json:"memory,omitempty"`
}

// RobotVDISpec defines the desired state of RobotVDI
type RobotVDISpec struct {
	Resources Resources `json:"resources,omitempty"`
	// ServiceType
	// +kubebuilder:validation:Enum=ClusterIP;NodePort
	// +kubebuilder:default="NodePort"
	ServiceType corev1.ServiceType `json:"serviceType,omitempty"`
	Ingress     bool               `json:"ingress,omitempty"`
	Privileged  bool               `json:"privileged,omitempty"`
	// NAT1TO1 for Neko.
	NAT1TO1 string `json:"nat1to1,omitempty"`
	// +kubebuilder:validation:Pattern=`^([0-9])+-([0-9])+$`
	// +kubebuilder:validation:Required
	WebRTCPortRange string `json:"webrtcPortRange,omitempty"`
	// VDI screen resolution options.
	// +kubebuilder:validation:Enum="2048x1152";"1920x1080";"1600x1200"
	// +kubebuilder:default="2048x1152"
	Resolution string `json:"resolution,omitempty"`
}

// RobotVDIStatus defines the observed state of RobotVDI
type RobotVDIStatus struct {
	Phase            RobotVDIPhase       `json:"phase,omitempty"`
	PodStatus        OwnedPodStatus      `json:"podStatus,omitempty"`
	ServiceTCPStatus OwnedResourceStatus `json:"serviceTCPStatus,omitempty"`
	ServiceUDPStatus OwnedResourceStatus `json:"serviceUDPStatus,omitempty"`
	IngressStatus    OwnedResourceStatus `json:"ingressStatus,omitempty"`
	PVCStatus        OwnedResourceStatus `json:"pvcStatus,omitempty"`
}
