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

type MetricType string

const (
	MetricTypeCPU         MetricType = "CPU"
	MetricTypeMemory      MetricType = "Memory"
	MetricTypeGPU         MetricType = "GPU"
	MetricTypeNetworkLoad MetricType = "NetworkLoad"
)

// MetricsCollectorSpec defines the desired state of MetricsCollector
type MetricsCollectorSpec struct {
	// TODO: Add default value
	// +kubebuilder:default=10
	WaitSeconds int64 `json:"waitSeconds"`
	CPU         bool  `json:"cpu"`
	Memory      bool  `json:"memory"`
	GPU         bool  `json:"gpu"`
	NetworkLoad bool  `json:"networkLoad"`
}

type CPUUtilization struct {
	Value          string `json:"value,omitempty"`
	HostPercentage string `json:"hostPercentage,omitempty"`
	CorePercentage string `json:"corePercentage,omitempty"`
	Message        string `json:"message,omitempty"`
}

type MemoryUtilization struct {
	Value          string `json:"value,omitempty"`
	HostPercentage string `json:"hostPercentage,omitempty"`
	Message        string `json:"message,omitempty"`
}

type NetworkLoad struct {
	Value string `json:"value,omitempty"`
	Load  string `json:"load,omitempty"`
}

type NetworkInterfaceUtilization struct {
	Name     string      `json:"name,omitempty"`
	Transmit NetworkLoad `json:"transmit,omitempty"`
	Receive  NetworkLoad `json:"receive,omitempty"`
}

type NetworkLoadUtilization struct {
	Interfaces []NetworkInterfaceUtilization `json:"interfaces,omitempty"`
	Message    string                        `json:"message,omitempty"`
}

type ComponentMetricStatus struct {
	OwnerResourceReference metav1.OwnerReference  `json:"ownerReference,omitempty"`
	PodReference           corev1.ObjectReference `json:"podReference,omitempty"`
	ContainerName          string                 `json:"containerName,omitempty"`
	CPUUtilization         CPUUtilization         `json:"cpuUtilization,omitempty"`
	MemoryUtilization      MemoryUtilization      `json:"memoryUtilization,omitempty"`
	NetworkLoadUtilization NetworkLoadUtilization `json:"networkLoadUtilization,omitempty"`
	Message                string                 `json:"message,omitempty"`
}

type MetricsCollectorPhase string

const (
	MetricsCollectorPhaseRobotNotFound MetricsCollectorPhase = "RobotNotFound"
	MetricsCollectorPhaseRunning       MetricsCollectorPhase = "Running"
)

// MetricsCollectorStatus defines the observed state of MetricsCollector
type MetricsCollectorStatus struct {
	LastUpdateTimestamp metav1.Time             `json:"lastUpdateTimestamp,omitempty"`
	ComponentMetrics    []ComponentMetricStatus `json:"componentMetrics,omitempty"`
	Allocatable         corev1.ResourceList     `json:"allocatable,omitempty"`
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
