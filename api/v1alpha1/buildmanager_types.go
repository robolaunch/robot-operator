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

type BuildManagerPhase string

const (
	BuildManagerRobotNotFound            BuildManagerPhase = "RobotNotFound"
	BuildManagerWaitingForOtherResources BuildManagerPhase = "WaitingForOtherResources"
	BuildManagerCreatingConfigMap        BuildManagerPhase = "CreatingConfigMap"
	BuildManagerBuildingRobot            BuildManagerPhase = "BuildingRobot"
	BuildManagerReady                    BuildManagerPhase = "Ready"
	BuildManagerFailed                   BuildManagerPhase = "Failed"
	BuildManagerDeactivating             BuildManagerPhase = "Deactivating"
	BuildManagerInactive                 BuildManagerPhase = "Inactive"
)

// BuildManagerStatus defines the observed state of BuildManager
type BuildManagerStatus struct {
	Phase                 BuildManagerPhase     `json:"phase,omitempty"`
	Active                bool                  `json:"active,omitempty"`
	ScriptConfigMapStatus ScriptConfigMapStatus `json:"scriptConfigMapStatus,omitempty"`
	Steps                 []StepStatus          `json:"steps,omitempty"`
}

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

func init() {
	SchemeBuilder.Register(&BuildManager{}, &BuildManagerList{})
}

func (buildmanager *BuildManager) GetConfigMapMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      buildmanager.Name + internal.CONFIG_MAP_SCRIPTS,
		Namespace: buildmanager.Namespace,
	}
}
