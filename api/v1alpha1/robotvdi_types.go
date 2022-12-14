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
	"github.com/robolaunch/robot-operator/internal"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

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
	MainDisplay bool               `json:"mainDisplay,omitempty"`
	// NAT1TO1 for Neko.
	NAT1TO1 string `json:"nat1to1,omitempty"`
	// +kubebuilder:validation:Pattern=`^([0-9])+-([0-9])+$`
	// +kubebuilder:validation:Required
	WebRTCPortRange string `json:"webrtcPortRange,omitempty"`
}

type RobotVDIPodStatus struct {
	Created bool            `json:"created,omitempty"`
	Phase   corev1.PodPhase `json:"phase,omitempty"`
	IP      string          `json:"ip,omitempty"`
}

type RobotVDIPVCStatus struct {
	Created bool `json:"created,omitempty"`
}

type RobotVDIServiceTCPStatus struct {
	Created bool `json:"created,omitempty"`
}

type RobotVDIServiceUDPStatus struct {
	Created bool `json:"created,omitempty"`
}

type RobotVDIIngressStatus struct {
	Created bool `json:"created,omitempty"`
}

type RobotVDIPhase string

const (
	RobotVDIPhaseCreatingPVC        RobotVDIPhase = "CreatingPVC"
	RobotVDIPhaseCreatingTCPService RobotVDIPhase = "CreatingTCPService"
	RobotVDIPhaseCreatingUDPService RobotVDIPhase = "CreatingUDPService"
	RobotVDIPhaseCreatingPod        RobotVDIPhase = "CreatingPod"
	RobotVDIPhaseCreatingIngress    RobotVDIPhase = "CreatingIngress"
	RobotVDIPhaseReady              RobotVDIPhase = "Running"
)

// RobotVDIStatus defines the observed state of RobotVDI
type RobotVDIStatus struct {
	Phase            RobotVDIPhase            `json:"phase,omitempty"`
	PodStatus        RobotVDIPodStatus        `json:"podStatus,omitempty"`
	ServiceTCPStatus RobotVDIServiceTCPStatus `json:"serviceTCPStatus,omitempty"`
	ServiceUDPStatus RobotVDIServiceUDPStatus `json:"serviceUDPStatus,omitempty"`
	IngressStatus    RobotVDIIngressStatus    `json:"ingressStatus,omitempty"`
	PVCStatus        RobotVDIPVCStatus        `json:"pvcStatus,omitempty"`
}

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

func init() {
	SchemeBuilder.Register(&RobotVDI{}, &RobotVDIList{})
}

func (robotvdi *RobotVDI) GetRobotVDIPVCMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.PVC_VDI_POSTFIX,
	}
}

func (robotvdi *RobotVDI) GetRobotVDIPodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.POD_VDI_POSTFIX,
	}
}

func (robotvdi *RobotVDI) GetRobotVDIServiceTCPMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.SVC_TCP_VDI_POSTFIX,
	}
}

func (robotvdi *RobotVDI) GetRobotVDIServiceUDPMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.SVC_UDP_VDI_POSTFIX,
	}
}

func (robotvdi *RobotVDI) GetRobotVDIIngressMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotvdi.Namespace,
		Name:      robotvdi.Name + internal.INGRESS_VDI_POSTFIX,
	}
}
