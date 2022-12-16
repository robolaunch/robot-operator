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
	"k8s.io/apimachinery/pkg/types"
)

// DiscoveryServerSpec defines the desired state of DiscoveryServer
type DiscoveryServerSpec struct {
	Attached  bool   `json:"attached,omitempty"`
	Cluster   string `json:"cluster,omitempty"`
	Hostname  string `json:"hostname,omitempty"`
	Subdomain string `json:"subdomain,omitempty"`
}

type DiscoveryServerServiceStatus struct {
	Created bool `json:"created,omitempty"`
}

type DiscoveryServerPodStatus struct {
	Created bool            `json:"created,omitempty"`
	Phase   corev1.PodPhase `json:"phase,omitempty"`
	IP      string          `json:"ip,omitempty"`
}

type DiscoveryServerConfigMapStatus struct {
	Created bool `json:"created,omitempty"`
}

type DiscoveryServerPhase string

const (
	DiscoveryServerPhaseCreatingService   DiscoveryServerPhase = "CreatingService"
	DiscoveryServerPhaseCreatingPod       DiscoveryServerPhase = "CreatingPod"
	DiscoveryServerPhaseCreatingConfigMap DiscoveryServerPhase = "CreatingConfigMap"
	DiscoveryServerPhaseReady             DiscoveryServerPhase = "Ready"
	DiscoveryServerPhaseDeletingConfigMap DiscoveryServerPhase = "DeletingConfigMap"
	DiscoveryServerPhaseDeletingPod       DiscoveryServerPhase = "DeletingPod"
	DiscoveryServerPhaseDeletingService   DiscoveryServerPhase = "DeletingService"
)

type ConnectionInfo struct {
	IP            string `json:"ip,omitempty"`
	ConfigMapName string `json:"configMapName,omitempty"`
}

// DiscoveryServerStatus defines the observed state of DiscoveryServer
type DiscoveryServerStatus struct {
	Phase           DiscoveryServerPhase           `json:"phase,omitempty"`
	ServiceStatus   DiscoveryServerServiceStatus   `json:"serviceStatus,omitempty"`
	PodStatus       DiscoveryServerPodStatus       `json:"podStatus,omitempty"`
	ConfigMapStatus DiscoveryServerConfigMapStatus `json:"configMapStatus,omitempty"`
	ConnectionInfo  ConnectionInfo                 `json:"connectionInfo,omitempty"`
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// DiscoveryServer is the Schema for the discoveryservers API
type DiscoveryServer struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   DiscoveryServerSpec   `json:"spec,omitempty"`
	Status DiscoveryServerStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// DiscoveryServerList contains a list of DiscoveryServer
type DiscoveryServerList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []DiscoveryServer `json:"items"`
}

func init() {
	SchemeBuilder.Register(&DiscoveryServer{}, &DiscoveryServerList{})
}

func (discoveryServer *DiscoveryServer) GetDiscoveryServerPodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      discoveryServer.Name,
		Namespace: discoveryServer.Namespace,
	}
}

func (discoveryServer *DiscoveryServer) GetDiscoveryServerServiceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      discoveryServer.Name + "-" + discoveryServer.Spec.Subdomain,
		Namespace: discoveryServer.Namespace,
	}
}

func (discoveryServer *DiscoveryServer) GetDiscoveryServerConfigMapMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      discoveryServer.Name,
		Namespace: discoveryServer.Namespace,
	}
}
