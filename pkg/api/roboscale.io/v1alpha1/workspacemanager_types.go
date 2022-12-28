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

// WorkspaceManagerSpec defines the desired state of WorkspaceManager
type WorkspaceManagerSpec struct {
	// Global path of workspaces. It's fixed to `/home/workspaces` path.
	WorkspacesPath string `json:"workspacesPath,omitempty"`
	// Workspace definitions of robot.
	// +kubebuilder:validation:MinItems=1
	Workspaces []Workspace `json:"workspaces,omitempty"`
}

type ClonerJobStatus struct {
	Created bool     `json:"created,omitempty"`
	Phase   JobPhase `json:"phase,omitempty"`
}

type WorkspaceManagerPhase string

const (
	WorkspaceManagerPhaseConfiguringWorkspaces WorkspaceManagerPhase = "ConfiguringWorkspaces"
	WorkspaceManagerPhaseReady                 WorkspaceManagerPhase = "Ready"
)

// WorkspaceManagerStatus defines the observed state of WorkspaceManager
type WorkspaceManagerStatus struct {
	Phase           WorkspaceManagerPhase `json:"phase,omitempty"`
	ClonerJobStatus ClonerJobStatus       `json:"clonerJobStatus,omitempty"`
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
