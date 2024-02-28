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

package v1alpha2

import (
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	logf "sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/webhook"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

// ********************************
// ROS2Workload webhooks
// ********************************

// log is for logging in this package.
var ros2workloadlog = logf.Log.WithName("ros2workload-resource")

func (r *ROS2Workload) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha2-ros2workload,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=ros2workloads,verbs=create;update,versions=v1alpha2,name=mros2workload.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &ROS2Workload{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *ROS2Workload) Default() {
	ros2workloadlog.Info("default", "name", r.Name)
}

// TODO(user): change verbs to "verbs=create;update;delete" if you want to enable deletion validation.
//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha2-ros2workload,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=ros2workloads,verbs=create;update,versions=v1alpha2,name=vros2workload.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &ROS2Workload{}

func (r *ROS2Workload) validateDiscoveryServerTemplate() error {

	discoveryServerTemplate := &robotv1alpha1.DiscoveryServer{
		ObjectMeta: metav1.ObjectMeta{
			Name:      r.GetDiscoveryServerMetadata().Name,
			Namespace: r.GetDiscoveryServerMetadata().Namespace,
		},
		Spec: r.Spec.DiscoveryServerTemplate,
	}

	discoveryServerTemplate.Default()
	return discoveryServerTemplate.ValidateCreate()
}

func (r *ROS2Workload) validateROS2BridgeTemplate() error {

	ros2BridgeTemplate := &ROS2Bridge{
		ObjectMeta: metav1.ObjectMeta{
			Name:      r.GetROS2BridgeMetadata().Name,
			Namespace: r.GetROS2BridgeMetadata().Namespace,
		},
		Spec: r.Spec.ROS2BridgeTemplate,
	}

	ros2BridgeTemplate.Default()
	return ros2BridgeTemplate.ValidateCreate()
}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *ROS2Workload) ValidateCreate() error {
	ros2workloadlog.Info("validate create", "name", r.Name)

	err := r.validateDiscoveryServerTemplate()
	if err != nil {
		return err
	}

	err = r.validateROS2BridgeTemplate()
	if err != nil {
		return err
	}

	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *ROS2Workload) ValidateUpdate(old runtime.Object) error {
	ros2workloadlog.Info("validate update", "name", r.Name)

	err := r.validateDiscoveryServerTemplate()
	if err != nil {
		return err
	}

	err = r.validateROS2BridgeTemplate()
	if err != nil {
		return err
	}

	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *ROS2Workload) ValidateDelete() error {
	ros2workloadlog.Info("validate delete", "name", r.Name)
	return nil
}
