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

type LaunchType string

const (
	LaunchTypeDistributed LaunchType = "Distributed"
	LaunchTypeDirect      LaunchType = "Direct"
)

// Prelaunch command or script is applied just before the node is started.
type Prelaunch struct {
	// Bash command to run before ROS node execution.
	// +kubebuilder:validation:Required
	Command string `json:"command"`
	// Script  string `json:"script,omitempty"`
}

// Launch description of a repository.
type Launch struct {
	// Name of the launch.
	// +kubebuilder:validation:Required
	Name string `json:"name"`
	// Name of the workspace.
	// +kubebuilder:validation:Required
	Workspace string `json:"workspace"`
	// +kubebuilder:validation:Required
	// +kubebuilder:validation:Enum=Distributed;Direct
	Type LaunchType `json:"type"`
	// Additional environment variables to set when launching ROS nodes.
	Env []corev1.EnvVar `json:"env,omitempty"`
	// Path to launchfile in repository. (eg. `linorobot/linorobot_gazebo/launch.py`)
	// +kubebuilder:validation:Required
	LaunchFilePath string `json:"launchFilePath"`
	// Launch parameters.
	Parameters map[string]string `json:"parameters,omitempty"`
	// Command or script to run just before node's execution.
	Prelaunch Prelaunch `json:"prelaunch,omitempty"`
}

// LaunchManagerSpec defines the desired state of LaunchManager
type LaunchManagerSpec struct {
	Launch []Launch `json:"launch,omitempty"`
}

type LaunchPodStatus struct {
	Created bool            `json:"created,omitempty"`
	Phase   corev1.PodPhase `json:"phase,omitempty"`
	IP      string          `json:"ip,omitempty"`
}

type LaunchManagerPhase string

const (
	LaunchManagerPhaseCreatingPod   LaunchManagerPhase = "Launching"
	LaunchManagerPhaseReady         LaunchManagerPhase = "Ready"
	LaunchManagerPhaseMalfunctioned LaunchManagerPhase = "Malfunctioned"
)

// LaunchManagerStatus defines the observed state of LaunchManager
type LaunchManagerStatus struct {
	Phase           LaunchManagerPhase `json:"phase,omitempty"`
	LaunchPodStatus LaunchPodStatus    `json:"launchPodStatus,omitempty"`
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// LaunchManager is the Schema for the launchmanagers API
type LaunchManager struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   LaunchManagerSpec   `json:"spec,omitempty"`
	Status LaunchManagerStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// LaunchManagerList contains a list of LaunchManager
type LaunchManagerList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []LaunchManager `json:"items"`
}

func init() {
	SchemeBuilder.Register(&LaunchManager{}, &LaunchManagerList{})
}
