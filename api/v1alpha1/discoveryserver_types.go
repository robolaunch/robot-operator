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

// DiscoveryServerSpec defines the desired state of DiscoveryServer
type DiscoveryServerSpec struct {
}

// DiscoveryServerStatus defines the observed state of DiscoveryServer
type DiscoveryServerStatus struct {
}

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
