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
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

// RobotDevSuiteSpec defines the desired state of RobotDevSuite
type RobotDevSuiteSpec struct {
	RobotVDITemplate RobotVDISpec `json:"robotVDITemplate,omitempty"`
	RobotIDETemplate RobotIDESpec `json:"robotIDETemplate,omitempty"`
}

type RobotVDIInstanceStatus struct {
	Created bool           `json:"created,omitempty"`
	Status  RobotVDIStatus `json:"status,omitempty"`
}

type RobotIDEInstanceStatus struct {
	Created bool           `json:"created,omitempty"`
	Status  RobotIDEStatus `json:"status,omitempty"`
}

type RobotDevSuitePhase string

const (
	RobotDevSuitePhaseCreatingRobotVDI = "CreatingRobotVDI"
	RobotDevSuitePhaseCreatingRobotIDE = "CreatingRobotIDE"
	RobotDevSuitePhaseRunning          = "Running"
)

// RobotDevSuiteStatus defines the observed state of RobotDevSuite
type RobotDevSuiteStatus struct {
	Phase          RobotDevSuitePhase     `json:"phase,omitempty"`
	RobotVDIStatus RobotVDIInstanceStatus `json:"robotVDIStatus,omitempty"`
	RobotIDEStatus RobotIDEInstanceStatus `json:"robotIDEStatus,omitempty"`
}

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

func init() {
	SchemeBuilder.Register(&RobotDevSuite{}, &RobotDevSuiteList{})
}
