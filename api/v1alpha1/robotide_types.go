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

// RobotIDESpec defines the desired state of RobotIDE
type RobotIDESpec struct {
	Resources Resources `json:"resources,omitempty"`
	// ServiceType
	// +kubebuilder:validation:Enum=ClusterIP;NodePort
	// +kubebuilder:default="NodePort"
	ServiceType corev1.ServiceType `json:"serviceType,omitempty"`
	Ingress     bool               `json:"ingress,omitempty"`
	Privileged  bool               `json:"privileged,omitempty"`
}

type RobotIDEPodStatus struct {
	Created bool            `json:"created,omitempty"`
	Phase   corev1.PodPhase `json:"phase,omitempty"`
	IP      string          `json:"ip,omitempty"`
}

type RobotIDEPhase string

const (
	RobotIDEPhaseCreatingService RobotIDEPhase = "CreatingService"
	RobotIDEPhaseCreatingPod     RobotIDEPhase = "CreatingPod"
	RobotIDEPhaseCreatingIngress RobotIDEPhase = "CreatingIngress"
	RobotIDEPhaseRunning         RobotIDEPhase = "Running"
)

type RobotIDEServiceStatus struct {
	Created bool `json:"created,omitempty"`
}

type RobotIDEIngressStatus struct {
	Created bool `json:"created,omitempty"`
}

// RobotIDEStatus defines the observed state of RobotIDE
type RobotIDEStatus struct {
	Phase         RobotIDEPhase         `json:"phase,omitempty"`
	PodStatus     RobotIDEPodStatus     `json:"podStatus,omitempty"`
	ServiceStatus RobotIDEServiceStatus `json:"serviceStatus,omitempty"`
	IngressStatus RobotIDEIngressStatus `json:"ingressStatus,omitempty"`
}

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

func init() {
	SchemeBuilder.Register(&RobotIDE{}, &RobotIDEList{})
}

func (robotide *RobotIDE) GetRobotIDEPodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotide.Namespace,
		Name:      robotide.Name + internal.POD_IDE_POSTFIX,
	}
}

func (robotide *RobotIDE) GetRobotIDEServiceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotide.Namespace,
		Name:      robotide.Name + internal.SVC_IDE_POSTFIX,
	}
}

func (robotide *RobotIDE) GetRobotIDEIngressMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Namespace: robotide.Namespace,
		Name:      robotide.Name + internal.INGRESS_IDE_POSTFIX,
	}
}
