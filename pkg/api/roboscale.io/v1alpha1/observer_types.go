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

func init() {
	SchemeBuilder.Register(&MetricsExporter{}, &MetricsExporterList{})
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// MetricsExporter is the Schema for the metricsexporters API
type MetricsExporter struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   MetricsExporterSpec   `json:"spec,omitempty"`
	Status MetricsExporterStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// MetricsExporterList contains a list of MetricsExporter
type MetricsExporterList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []MetricsExporter `json:"items"`
}

// ********************************
// MetricsExporter types
// ********************************

// MetricsExporterSpec defines the desired state of MetricsExporter
type MetricsExporterSpec struct {
	Foo string `json:"foo,omitempty"`
}

// MetricsExporterStatus defines the observed state of MetricsExporter
type MetricsExporterStatus struct {
}
