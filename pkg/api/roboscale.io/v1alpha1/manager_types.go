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

// WorkspaceManager configures the ROS 2 workspaces and repositories by executing Kubernetes jobs.
type WorkspaceManager struct {
	metav1.TypeMeta `json:",inline"`
	// Standard object's metadata.
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the WorkspaceManager.
	Spec WorkspaceManagerSpec `json:"spec,omitempty"`
	// Most recently observed status of the WorkspaceManager.
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
	metav1.TypeMeta `json:",inline"`
	// Standard object's metadata.
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the BuildManager.
	Spec BuildManagerSpec `json:"spec,omitempty"`
	// Most recently observed status of the BuildManager.
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
	metav1.TypeMeta `json:",inline"`
	// Standard object's metadata.
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the LaunchManager.
	Spec LaunchManagerSpec `json:"spec,omitempty"`
	// Most recently observed status of the LaunchManager.
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
	// [*Autofilled*] Absolute path of repository
	Path string `json:"path,omitempty"`
	// [*Autofilled*] User or organization, maintainer of repository
	Owner string `json:"owner,omitempty"`
	// [*Autofilled*] Repository name
	Repo string `json:"repo,omitempty"`
	// [*Autofilled*] Hash of last commit
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

// WorkspaceManagerSpec defines the desired state of WorkspaceManager.
type WorkspaceManagerSpec struct {
	// Global path of workspaces. It's fixed to `/root/workspaces` path.
	WorkspacesPath string `json:"workspacesPath,omitempty"`
	// Workspace definitions of robot.
	// Multiple ROS 2 workspaces can be configured over this field.
	// +kubebuilder:validation:MinItems=1
	Workspaces []Workspace `json:"workspaces,omitempty"`
	// WorkspaceManager is triggered if this field is set to `true`.
	// Then the workspaces are being configured again while backing up the old configurations.
	// This field is often used by operator.
	UpdateNeeded bool `json:"updateNeeded,omitempty"`
}

// WorkspaceManagerStatus defines the observed state of WorkspaceManager.
type WorkspaceManagerStatus struct {
	// Phase of WorkspaceManager.
	Phase WorkspaceManagerPhase `json:"phase,omitempty"`
	// Status of cloner job.
	ClonerJobStatus OwnedResourceStatus `json:"clonerJobStatus,omitempty"`
	// Status of cleanup jobs that runs while reconfiguring workspaces.
	CleanupJobStatus OwnedResourceStatus `json:"cleanupJobStatus,omitempty"`
	// Incremental version of workspace configuration map.
	// Used to determine changes in configuration.
	Version int `json:"version,omitempty"`
}

// ********************************
// BuildManager types
// ********************************

// Step is a command or script to execute when building a robot. Either `command` or `script` should be specified
// for each step.
type Step struct {
	// Cluster selector.
	// If the current instance name is on the list, BuildManager creates building jobs.
	Instances []string `json:"instances,omitempty"`
	// Name of the step.
	Name string `json:"name"`
	// Name of the workspace.
	// Should be selected among the existing workspaces in WorkspaceManager's manifests.
	Workspace string `json:"workspace"`
	// Bash command to run.
	// Assume that your command will be `/bin/bash -c <COMMAND>`.
	// Use logical operators (eg. `&&`) and pipes if the multiple dependent commands will be executed.
	Command string `json:"command,omitempty"`
	// Bash script to run.
	Script string `json:"script,omitempty"`
	// Environment variables for step.
	Env []corev1.EnvVar `json:"env,omitempty"`
}

// BuildManagerSpec defines the desired state of BuildManager.
type BuildManagerSpec struct {
	// Defines the building steps.
	Steps []Step `json:"steps,omitempty"`
}

// BuildManagerStatus defines the observed state of BuildManager.
type BuildManagerStatus struct {
	// Phase of BuildManager.
	Phase BuildManagerPhase `json:"phase,omitempty"`
	// Indicates if the BuildManager is currently executing it's jobs.
	Active bool `json:"active,omitempty"`
	// Status of the ConfigMap that holds scripts.
	// If a script is specified inside `.spec.steps[k]`, they are mounted to the step jobs via this ConfigMap.
	ScriptConfigMapStatus OwnedResourceStatus `json:"scriptConfigMapStatus,omitempty"`
	// Statuses of the build steps.
	Steps []StepStatus `json:"steps,omitempty"`
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

type LaunchType string

const (
	LaunchTypeLaunch LaunchType = "Launch"
	LaunchTypeRun    LaunchType = "Run"
	LaunchTypeCustom LaunchType = "Custom"
)

// Launch description of a repository.
type Launch struct {
	// Cluster selector.
	// If the current instance name is on the list, LaunchManager creates launch pods.
	Instances []string `json:"instances,omitempty"`
	// Name of the workspace.
	// Should be selected among the existing workspaces in WorkspaceManager's manifests.
	// +kubebuilder:validation:Required
	Workspace string `json:"workspace"`
	// ROS 2 namespacing. May not be suitable for all launchfiles.
	// If used, all the node names and topic names should be defined relative, not absolute.
	// (eg. `cmd_vel` instead of /cmd_vel``)
	// +kubebuilder:validation:Required
	Namespacing bool `json:"namespacing,omitempty"`
	// Launching type. Can be `Launch`, `Run` or `Custom`.
	// +kubebuilder:validation:Enum=Launch;Run;Custom
	Type LaunchType `json:"type,omitempty"`
	// Package name. (eg. `robolaunch_cloudy_navigation`)
	// +kubebuilder:validation:Required
	Package string `json:"package"`
	// Launchfile. (eg. `nav_launch.py`)
	// Required and used if the launch type is `Launch`.
	Launchfile string `json:"launchfile"`
	// Executable file name. (eg. `webcam_pub.py`)
	// Required if the launch type is `Run`.
	Executable string `json:"executable"`
	// Additional environment variables to set when launching ROS nodes.
	Env []corev1.EnvVar `json:"env,omitempty"`
	// Launch parameters.
	Parameters map[string]string `json:"parameters,omitempty"`
	// Command or script to run just before node's execution.
	Prelaunch Prelaunch `json:"prelaunch,omitempty"`
	// Launch container privilege.
	Privileged bool `json:"privileged,omitempty"`
	// Launch container resource limits.
	Resources Resources `json:"resources,omitempty"`
}

// LaunchManagerSpec defines the desired state of LaunchManager.
type LaunchManagerSpec struct {
	// Launch processes connects an X11 socket if it's set to `true` and a target RobotVDI resource is set in labels with key `robolaunch.io/target-vdi`.
	// Applications that requires GUI can be executed such as rViz.
	Display bool `json:"display,omitempty"`
	// Launch descriptions.
	// Every object defined here generates a launching command in the specified workspace.
	Launch map[string]Launch `json:"launch,omitempty"`
}

type LaunchStatus struct {
	// Inditaces if the launch process are actively running on cluster.
	// It may not be selected by launch cluster selectors.
	Active bool `json:"active,omitempty"`
	// Statuses of the containers of pods owned by LaunchManager.
	ContainerStatus corev1.ContainerStatus `json:"containerStatus,omitempty"`
}

type LaunchPodStatus struct {
	// Launch pod status. Every LaunchManager creates one pod if active.
	Status OwnedPodStatus `json:"status,omitempty"`
	// Status of launch objects. Every launch object generates a `ros2 launch` command that will run as an entrypoint in a container.
	LaunchStatus map[string]LaunchStatus `json:"launchStatus,omitempty"`
}

// LaunchManagerStatus defines the observed state of LaunchManager.
type LaunchManagerStatus struct {
	// Phase of LaunchManager.
	Phase LaunchManagerPhase `json:"phase,omitempty"`
	// Indicates if the LaunchManager is attached to a Robot and actively running.
	Active bool `json:"active,omitempty"`
	// Collective statuses of launch pod and launch objects.
	LaunchPodStatus LaunchPodStatus `json:"launchPodStatus,omitempty"`
}
