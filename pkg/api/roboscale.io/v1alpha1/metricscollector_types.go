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

// MetricsCollectorSpec defines the desired state of MetricsCollector
type MetricsCollectorSpec struct {
}

// MetricsCollectorStatus defines the observed state of MetricsCollector
type MetricsCollectorStatus struct {
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// MetricsCollector is the Schema for the metricscollectors API
type MetricsCollector struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   MetricsCollectorSpec   `json:"spec,omitempty"`
	Status MetricsCollectorStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// MetricsCollectorList contains a list of MetricsCollector
type MetricsCollectorList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []MetricsCollector `json:"items"`
}

func init() {
	SchemeBuilder.Register(&MetricsCollector{}, &MetricsCollectorList{})
}
