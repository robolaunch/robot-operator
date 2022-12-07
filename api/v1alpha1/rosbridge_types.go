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

// ROSBridgeSpec defines the desired state of ROSBridge
type ROSBridgeSpec struct {
}

// ROSBridgeStatus defines the observed state of ROSBridge
type ROSBridgeStatus struct {
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// ROSBridge is the Schema for the rosbridges API
type ROSBridge struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   ROSBridgeSpec   `json:"spec,omitempty"`
	Status ROSBridgeStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// ROSBridgeList contains a list of ROSBridge
type ROSBridgeList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []ROSBridge `json:"items"`
}

func init() {
	SchemeBuilder.Register(&ROSBridge{}, &ROSBridgeList{})
}
