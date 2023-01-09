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
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

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

func init() {
	SchemeBuilder.Register(&WorkspaceManager{}, &WorkspaceManagerList{})
}

func (workspacemanager *WorkspaceManager) GetClonerJobMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      workspacemanager.Name + internal.JOB_CLONER_POSTFIX,
		Namespace: workspacemanager.Namespace,
	}
}

func (workspacemanager *WorkspaceManager) GetCleanupJobMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      workspacemanager.Name + internal.JOB_CLEANUP_POSTFIX,
		Namespace: workspacemanager.Namespace,
	}
}
