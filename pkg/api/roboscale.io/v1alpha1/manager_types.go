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
	SchemeBuilder.Register(&WorkspaceManager{}, &WorkspaceManagerList{})
	SchemeBuilder.Register(&BuildManager{}, &BuildManagerList{})
	SchemeBuilder.Register(&LaunchManager{}, &LaunchManagerList{})
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// WorkspaceManager is the Schema for the workspacemanagers API
type WorkspaceManager struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   WorkspaceManagerSpec   `json:"spec,omitempty"`
	Status WorkspaceManagerStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// WorkspaceManagerList contains a list of WorkspaceManager
type WorkspaceManagerList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []WorkspaceManager `json:"items"`
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// BuildManager is the Schema for the buildmanagers API
type BuildManager struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   BuildManagerSpec   `json:"spec,omitempty"`
	Status BuildManagerStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// BuildManagerList contains a list of BuildManager
type BuildManagerList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []BuildManager `json:"items"`
}

//+genclient
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

// ********************************
// WorkspaceManager types
// ********************************

// Repository description.
type Repository struct {
	// Base URL of the repository.
	// +kubebuilder:validation:Required
	URL string `json:"url"`
	// Branch of the repository to clone.
	// +kubebuilder:validation:Required
	Branch string `json:"branch"`
	// [Autofilled] Absolute path of repository
	Path string `json:"path,omitempty"`
	// [Autofilled] User or organization, maintainer of repository
	Owner string `json:"owner,omitempty"`
	// [Autofilled] Repository name
	Repo string `json:"repo,omitempty"`
	// [Autofilled] Hash of last commit
	Hash string `json:"hash,omitempty"`
}

// Workspace description. Each robot should contain at least one workspace. A workspace should contain at least one
// repository in it.
type Workspace struct {
	// Name of workspace. If a workspace's name is `my_ws`, it's absolute path is `/home/workspaces/my_ws`.
	// +kubebuilder:validation:Required
	Name string `json:"name"`
	// +kubebuilder:validation:Required
	Distro ROSDistro `json:"distro"`
	// Repositories to clone inside workspace's `src` directory.
	Repositories map[string]Repository `json:"repositories"`
}

// WorkspaceManagerSpec defines the desired state of WorkspaceManager
type WorkspaceManagerSpec struct {
	// Global path of workspaces. It's fixed to `/home/workspaces` path.
	WorkspacesPath string `json:"workspacesPath,omitempty"`
	// Workspace definitions of robot.
	// +kubebuilder:validation:MinItems=1
	Workspaces []Workspace `json:"workspaces,omitempty"`
	// Need update
	UpdateNeeded bool `json:"updateNeeded,omitempty"`
}

type ClonerJobStatus struct {
	Created bool     `json:"created,omitempty"`
	Phase   JobPhase `json:"phase,omitempty"`
}

type CleanupJobStatus struct {
	Created bool     `json:"created,omitempty"`
	Phase   JobPhase `json:"phase,omitempty"`
}

// WorkspaceManagerStatus defines the observed state of WorkspaceManager
type WorkspaceManagerStatus struct {
	Phase            WorkspaceManagerPhase `json:"phase,omitempty"`
	ClonerJobStatus  ClonerJobStatus       `json:"clonerJobStatus,omitempty"`
	CleanupJobStatus CleanupJobStatus      `json:"cleanupJobStatus,omitempty"`
	Version          int                   `json:"version,omitempty"`
}

// ********************************
// BuildManager types
// ********************************

// Step is a command or script to execute when building a robot. Either `command` or `script` should be specified
// for each step.
type Step struct {
	// Name of the step.
	Name string `json:"name"`
	// Name of the workspace.
	Workspace string `json:"workspace"`
	// Bash command to run.
	Command string `json:"command,omitempty"`
	// Bash script to run.
	Script string `json:"script,omitempty"`
	// Environment variables for step.
	Env []corev1.EnvVar `json:"env,omitempty"`
}

// BuildManagerSpec defines the desired state of BuildManager
type BuildManagerSpec struct {
	Steps []Step `json:"steps,omitempty"`
}

type StepStatus struct {
	Step       Step     `json:"step,omitempty"`
	JobName    string   `json:"jobName,omitempty"`
	JobCreated bool     `json:"created,omitempty"`
	JobPhase   JobPhase `json:"jobPhase,omitempty"`
}

type ScriptConfigMapStatus struct {
	Created bool `json:"created,omitempty"`
}

// BuildManagerStatus defines the observed state of BuildManager
type BuildManagerStatus struct {
	Phase                 BuildManagerPhase     `json:"phase,omitempty"`
	Active                bool                  `json:"active,omitempty"`
	ScriptConfigMapStatus ScriptConfigMapStatus `json:"scriptConfigMapStatus,omitempty"`
	Steps                 []StepStatus          `json:"steps,omitempty"`
}

// ********************************
// LaunchManager types
// ********************************

// Prelaunch command or script is applied just before the node is started.
type Prelaunch struct {
	// Bash command to run before ROS node execution.
	// +kubebuilder:validation:Required
	Command string `json:"command"`
	// Script  string `json:"script,omitempty"`
}

// Launch description of a repository.
type Launch struct {
	// Cluster selector.
	Selector map[string]string `json:"selector,omitempty"`
	// Name of the workspace.
	// +kubebuilder:validation:Required
	Workspace string `json:"workspace"`
	// Name of the repository.
	// +kubebuilder:validation:Required
	Repository string `json:"repository"`
	// Name of the repository.
	// +kubebuilder:validation:Required
	Namespacing bool `json:"namespacing,omitempty"`
	// Additional environment variables to set when launching ROS nodes.
	Env []corev1.EnvVar `json:"env,omitempty"`
	// Path to launchfile in repository. (eg. `linorobot/linorobot_gazebo/launch.py`)
	// +kubebuilder:validation:Required
	LaunchFilePath string `json:"launchFilePath"`
	// Launch parameters.
	Parameters map[string]string `json:"parameters,omitempty"`
	// Command or script to run just before node's execution.
	Prelaunch Prelaunch `json:"prelaunch,omitempty"`
	// Launch container privilege.
	Privileged bool `json:"privileged,omitempty"`
}

// LaunchManagerSpec defines the desired state of LaunchManager
type LaunchManagerSpec struct {
	Launch map[string]Launch `json:"launch,omitempty"`
}

type LaunchStatus struct {
	Active          bool                   `json:"active,omitempty"`
	ContainerStatus corev1.ContainerStatus `json:"containerStatus,omitempty"`
}

type LaunchPodStatus struct {
	Created      bool                    `json:"created,omitempty"`
	Phase        corev1.PodPhase         `json:"phase,omitempty"`
	IP           string                  `json:"ip,omitempty"`
	LaunchStatus map[string]LaunchStatus `json:"launchStatus,omitempty"`
}

// LaunchManagerStatus defines the observed state of LaunchManager
type LaunchManagerStatus struct {
	Phase           LaunchManagerPhase `json:"phase,omitempty"`
	Active          bool               `json:"active,omitempty"`
	LaunchPodStatus LaunchPodStatus    `json:"launchPodStatus,omitempty"`
}
