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
//+kubebuilder:printcolumn:name="VDI",type=string,JSONPath=`.spec.vdiEnabled`
//+kubebuilder:printcolumn:name="IDE",type=string,JSONPath=`.spec.ideEnabled`
//+kubebuilder:printcolumn:name="Remote IDE RS",type=string,JSONPath=`.spec.remoteIDEEnabled`
//+kubebuilder:printcolumn:name="Phase",type=string,JSONPath=`.status.phase`

// RobotDevSuite is a custom resource that creates dynamically configured
// development environments for robots.
type RobotDevSuite struct {
	metav1.TypeMeta `json:",inline"`
	// Standard object's metadata.
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the RobotDevSuite.
	Spec RobotDevSuiteSpec `json:"spec,omitempty"`
	// Most recently observed status of the RobotDevSuite.
	Status RobotDevSuiteStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// RobotDevSuiteList contains a list of RobotDevSuite.
type RobotDevSuiteList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []RobotDevSuite `json:"items"`
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status
//+kubebuilder:printcolumn:name="Display",type=string,JSONPath=`.spec.display`
//+kubebuilder:printcolumn:name="Privileged",type=string,JSONPath=`.spec.privileged`
//+kubebuilder:printcolumn:name="GPU",type=string,JSONPath=`.spec.resources.gpuCore`
//+kubebuilder:printcolumn:name="Phase",type=string,JSONPath=`.status.phase`

// RobotIDE creates and manages Cloud IDE resources and workloads.
type RobotIDE struct {
	metav1.TypeMeta `json:",inline"`
	// Standard object's metadata.
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the RobotIDE.
	Spec RobotIDESpec `json:"spec,omitempty"`
	// Most recently observed status of the RobotIDE.
	Status RobotIDEStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// RobotIDEList contains a list of RobotIDE.
type RobotIDEList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []RobotIDE `json:"items"`
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status
//+kubebuilder:printcolumn:name="Resolution",type=string,JSONPath=`.spec.resolution`
//+kubebuilder:printcolumn:name="Ports",type=string,JSONPath=`.spec.webrtcPortRange`
//+kubebuilder:printcolumn:name="Privileged",type=string,JSONPath=`.spec.privileged`
//+kubebuilder:printcolumn:name="GPU",type=string,JSONPath=`.spec.resources.gpuCore`
//+kubebuilder:printcolumn:name="Phase",type=string,JSONPath=`.status.phase`

// RobotVDI creates and manages Cloud VDI resources and workloads.
type RobotVDI struct {
	metav1.TypeMeta `json:",inline"`
	// Standard object's metadata.
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the RobotVDI.
	Spec RobotVDISpec `json:"spec,omitempty"`
	// Most recently observed status of the RobotVDI.
	Status RobotVDIStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// RobotVDIList contains a list of RobotVDI.
type RobotVDIList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []RobotVDI `json:"items"`
}

// ********************************
// RobotDevSuite types
// ********************************

// RobotDevSuiteSpec defines the desired state of RobotDevSuite.
type RobotDevSuiteSpec struct {
	// If `true`, a Cloud VDI will be provisioned inside development suite.
	VDIEnabled bool `json:"vdiEnabled,omitempty"`
	// Configurational parameters of RobotVDI. Applied if `.spec.vdiEnabled` is set to `true`.
	RobotVDITemplate RobotVDISpec `json:"robotVDITemplate,omitempty"`
	// If `true`, a Cloud IDE will be provisioned inside development suite.
	IDEEnabled bool `json:"ideEnabled,omitempty"`
	// Configurational parameters of RobotIDE. Applied if `.spec.ideEnabled` is set to `true`.
	RobotIDETemplate RobotIDESpec `json:"robotIDETemplate,omitempty"`
	// If `true`, a relay server for remote Cloud IDE will be provisioned inside development suite.
	RemoteIDEEnabled bool `json:"remoteIDEEnabled,omitempty"`
	// Configurational parameters of remote IDE. Applied if `.spec.remoteIDEEnabled` is set to `true`.
	RemoteIDERelayServerTemplate RelayServerSpec `json:"remoteIDERelayServerTemplate,omitempty"`
}

// RobotDevSuiteStatus defines the observed state of RobotDevSuite.
type RobotDevSuiteStatus struct {
	// Phase of RobotDevSuite.
	Phase RobotDevSuitePhase `json:"phase,omitempty"`
	// Status of RobotVDI.
	RobotVDIStatus OwnedRobotServiceStatus `json:"robotVDIStatus,omitempty"`
	// Status of RobotIDE.
	RobotIDEStatus OwnedRobotServiceStatus `json:"robotIDEStatus,omitempty"`
	// Status of remote Cloud IDE RelayServer. Created only if the instance type is Physical Instance.
	RemoteIDERelayServerStatus OwnedRobotServiceStatus `json:"remoteIDERelayServerStatus,omitempty"`
	// [*alpha*] Indicates if RobotDevSuite is attached to a Robot and actively provisioned it's resources.
	Active bool `json:"active,omitempty"`
}

// ********************************
// RobotIDE types
// ********************************

// RobotIDESpec defines the desired state of RobotIDE.
type RobotIDESpec struct {
	// Resource limitations of Cloud IDE.
	Resources Resources `json:"resources,omitempty"`
	// Service type of Cloud IDE. `ClusterIP` and `NodePort` is supported.
	// +kubebuilder:validation:Enum=ClusterIP;NodePort
	// +kubebuilder:default="NodePort"
	ServiceType corev1.ServiceType `json:"serviceType,omitempty"`
	// If `true`, containers of RobotIDE will be privileged containers.
	// It can be used in physical instances where it's necessary to access
	// I/O devices on the host machine.
	// Not recommended to activate this field on cloud instances.
	Privileged bool `json:"privileged,omitempty"`
	// Cloud IDE connects an X11 socket if it's set to `true` and a target RobotVDI resource is set in labels with key `robolaunch.io/target-vdi`.
	// Applications that requires GUI can be executed such as rViz.
	Display bool `json:"display,omitempty"`
	// [*alpha*] RobotIDE will create an Ingress resource if `true`.
	Ingress bool `json:"ingress,omitempty"`
}

// RobotIDEStatus defines the observed state of RobotIDE.
type RobotIDEStatus struct {
	// Phase of RobotIDE.
	Phase RobotIDEPhase `json:"phase,omitempty"`
	// Status of Cloud IDE pod.
	PodStatus OwnedPodStatus `json:"podStatus,omitempty"`
	// Status of Cloud IDE service.
	ServiceStatus OwnedServiceStatus `json:"serviceStatus,omitempty"`
	// Status of Cloud IDE Ingress.
	IngressStatus OwnedResourceStatus `json:"ingressStatus,omitempty"`
	// Status of Cloud IDE ServiceExport. Created only if the instance type is Physical Instance.
	ServiceExportStatus OwnedResourceStatus `json:"serviceExportStatus,omitempty"`
	// Status of Cloud IDE service for custom ports. Created only if the robot has an additional config with key `IDE_CUSTOM_PORT_RANGE`.
	CustomPortServiceStatus OwnedServiceStatus `json:"customPortServiceStatus,omitempty"`
	// Status of Cloud IDE ingress for custom ports service. Created only if the robot has an additional config with key `IDE_CUSTOM_PORT_RANGE` and `.spec.ingress` is `true`.
	CustomPortIngressStatus OwnedResourceStatus `json:"customPortIngressStatus,omitempty"`
}

// ********************************
// RobotVDI types
// ********************************

// VDI resource limits.
type Resources struct {
	// GPU instance that will be allocated. eg. nvidia.com/mig-1g.5gb. Defaults to "nvidia.com/gpu".
	// +kubebuilder:default="nvidia.com/gpu"
	GPUInstance string `json:"gpuInstance,omitempty"`
	// GPU core number that will be allocated.
	GPUCore int `json:"gpuCore,omitempty"`
	// CPU resource limit.
	// +kubebuilder:validation:Pattern=`^([0-9])+(m)$`
	CPU string `json:"cpu,omitempty"`
	// Memory resource limit.
	// +kubebuilder:validation:Pattern=`^([0-9])+(Mi|Gi)$`
	Memory string `json:"memory,omitempty"`
}

// RobotVDISpec defines the desired state of RobotVDI.
type RobotVDISpec struct {
	// Resource limitations of Cloud IDE.
	Resources Resources `json:"resources,omitempty"`
	// Service type of Cloud IDE. `ClusterIP` and `NodePort` is supported.
	// +kubebuilder:validation:Enum=ClusterIP;NodePort
	// +kubebuilder:default="NodePort"
	ServiceType corev1.ServiceType `json:"serviceType,omitempty"`
	// If `true`, containers of RobotIDE will be privileged containers.
	// It can be used in physical instances where it's necessary to access
	// I/O devices on the host machine.
	// Not recommended to activate this field on cloud instances.
	Privileged bool `json:"privileged,omitempty"`
	// NAT1TO1 option for Cloud VDI.
	NAT1TO1 string `json:"nat1to1,omitempty"`
	// UDP port range to used in WebRTC connections.
	// +kubebuilder:validation:Pattern=`^([0-9])+-([0-9])+$`
	// +kubebuilder:validation:Required
	WebRTCPortRange string `json:"webrtcPortRange,omitempty"`
	// VDI screen resolution options. Default is `2048x1152`.
	// +kubebuilder:validation:Enum="2048x1152";"1920x1080";"1600x1200"
	// +kubebuilder:default="2048x1152"
	Resolution string `json:"resolution,omitempty"`
	// [*alpha*] RobotIDE will create an Ingress resource if `true`.
	Ingress bool `json:"ingress,omitempty"`
	// If true, VDI uses plain h264 instead of nvh264enc.
	DisableNVENC bool `json:"disableNvenc,omitempty"`
}

// RobotVDIStatus defines the observed state of RobotVDI.
type RobotVDIStatus struct {
	// Phase of RobotVDI.
	Phase RobotVDIPhase `json:"phase,omitempty"`
	// Status of Cloud VDI pod.
	PodStatus OwnedPodStatus `json:"podStatus,omitempty"`
	// Status of Cloud VDI TCP service.
	ServiceTCPStatus OwnedServiceStatus `json:"serviceTCPStatus,omitempty"`
	// Status of Cloud VDI UDP service.
	ServiceUDPStatus OwnedResourceStatus `json:"serviceUDPStatus,omitempty"`
	// Status of Cloud VDI Ingress.
	IngressStatus OwnedResourceStatus `json:"ingressStatus,omitempty"`
	// Status of Cloud VDI persistent volume claim.
	// This PVC dynamically provisions a volume that is a shared
	// between RobotVDI workloads and other workloads that requests
	// display.
	PVCStatus OwnedResourceStatus `json:"pvcStatus,omitempty"`
}
