package v1alpha1

import (
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

// RobotSpec defines the desired state of Robot
type RobotSpec struct {
}

// RobotStatus defines the observed state of Robot
type RobotStatus struct {
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// Robot is the Schema for the robots API
type Robot struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   RobotSpec   `json:"spec,omitempty"`
	Status RobotStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// RobotList contains a list of Robot
type RobotList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []Robot `json:"items"`
}

func init() {
	SchemeBuilder.Register(&Robot{}, &RobotList{})
}
