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

// MetricsExporter collects metrics from host machine and expose them
// from the Kubernetes API.
type MetricsExporter struct {
	metav1.TypeMeta `json:",inline"`
	// Standard object's metadata.
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the MetricsExporter.
	Spec MetricsExporterSpec `json:"spec,omitempty"`
	// Most recently observed status of the MetricsExporter.
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

type GPUMetrics struct {
	// MetricsExporter watches volatile GPU usage in the host machine
	// if it's set to `true`.
	Track bool `json:"track,omitempty"`
	// Watching latency.
	Interval int `json:"interval,omitempty"`
}

type NetworkMetrics struct {
	// MetricsExporter watches network loads in the host machine
	// if it's set to `true`.
	Track bool `json:"track,omitempty"`
	// Watching latency.
	Interval int `json:"interval,omitempty"`
	// Network interfaces which are desired to being watched.
	Interfaces []string `json:"interfaces,omitempty"`
}

type StorageMetrics struct {
	// MetricsExporter watches storage usage in the host machine
	// if it's set to `true`.
	Track bool `json:"track,omitempty"`
	// Watching latency.
	Interval int `json:"interval,omitempty"`
}

// MetricsExporterSpec defines the desired state of MetricsExporter.
type MetricsExporterSpec struct {
	// Configurational parameters about GPU metrics collection.
	GPU GPUMetrics `json:"gpu,omitempty"`
	// Configurational parameters about network metrics collection.
	Network NetworkMetrics `json:"network,omitempty"`
	// Configurational parameters about storage metrics collection.
	Storage StorageMetrics `json:"storage,omitempty"`
}

// Current usage of GPU cores belongs to a GPU instance (eg. mig-1g.10gb).
type GPUInstanceStatus struct {
	// Number of total virtual cores.
	Capacity string `json:"capacity,omitempty"`
	// Number of allocated virtual cores.
	// +kubebuilder:default="0"
	Allocated string `json:"allocated,omitempty"`
}

type GPUUtilizationStatus struct {
	// Volatile GPU utilization. Shows a percentage gathered from `nvidia-smi` command.
	Utilization string `json:"utilization,omitempty"`
	// Last update time.
	LastUpdateTimestamp string `json:"lastUpdateTimestamp,omitempty"`
}

type NetworkInterfaceLoad struct {
	// Average load of incoming packets.
	IncomingLoad string `json:"in,omitempty"`
	// Average load of outgoing packets.
	OutgoingLoad string `json:"out,omitempty"`
}

type NetworkLoadStatus struct {
	// Loads values of network interfaces.
	Load map[string]NetworkInterfaceLoad `json:"load,omitempty"`
	// Last update time.
	LastUpdateTimestamp string `json:"lastUpdateTimestamp,omitempty"`
}

type StorageUsage struct {
	// Size of the filesystem.
	Size string `json:"size,omitempty"`
	// Size of the used parts of a filesystem.
	Used string `json:"used,omitempty"`
	// Usage percentage of a filesystem.
	Percentage string `json:"percentage,omitempty"`
	// Directory that the filesystem mounted on.
	MountedOn string `json:"mountedOn,omitempty"`
}

type StorageStatus struct {
	// Total usage percentage of all filesystems.
	TotalPercentage string `json:"totalPercentage,omitempty"`
	// Usage values of filesystems.
	Usage map[string]StorageUsage `json:"storage,omitempty"`
	// Last update time.
	LastUpdateTimestamp string `json:"lastUpdateTimestamp,omitempty"`
}

type Usage struct {
	// GPU model
	GPUModel string `json:"gpuModel,omitempty"`
	// GPU usage information.
	// Will be deprecated after implementing checks for each GPU instance.
	GPU GPUUtilizationStatus `json:"gpu,omitempty"`
	// GPU virtual cores.
	GPUInstanceUsage map[string]GPUInstanceStatus `json:"gpuInstanceUsage,omitempty"`
	// Network usage information.
	Network NetworkLoadStatus `json:"network,omitempty"`
	// Storage usage information
	Storage StorageStatus `json:"storage,omitempty"`
}

type MetricsExporterPhase string

const (
	MetricsExporterPhaseCreatingRole           MetricsExporterPhase = "CreatingRole"
	MetricsExporterPhaseCreatingServiceAccount MetricsExporterPhase = "CreatingServiceAccount"
	MetricsExporterPhaseCreatingRoleBinding    MetricsExporterPhase = "CreatingRoleBinding"
	MetricsExporterPhaseCreatingPod            MetricsExporterPhase = "CreatingPod"
	MetricsExporterPhaseReady                  MetricsExporterPhase = "Ready"
)

// MetricsExporterStatus defines the observed state of MetricsExporter.
type MetricsExporterStatus struct {
	// Phase of MetricsExporter.
	Phase MetricsExporterPhase `json:"phase,omitempty"`
	// Status of role created for main and sidecar applications.
	RoleStatus OwnedResourceStatus `json:"roleStatus,omitempty"`
	// Status of role binding created for main and sidecar applications.
	RoleBindingStatus OwnedResourceStatus `json:"roleBindingStatus,omitempty"`
	// Status of service account created for main and sidecar applications.
	ServiceAccountStatus OwnedResourceStatus `json:"saStatus,omitempty"`
	// Status of MetricsExporter pod.
	PodStatus OwnedResourceStatus `json:"podStatus,omitempty"`
	// Usage metrics.
	Usage Usage `json:"usage,omitempty"`
}
