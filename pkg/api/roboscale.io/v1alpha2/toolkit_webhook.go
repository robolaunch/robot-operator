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
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	logf "sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
)

// ********************************
// ROS2Bridge webhooks
// ********************************

// log is for logging in this package.
var ros2bridgelog = logf.Log.WithName("ros2bridge-resource")

func (r *ROS2Bridge) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha2-ros2bridge,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=ros2bridges,verbs=create;update,versions=v1alpha2,name=mros2bridge.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &ROS2Bridge{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *ROS2Bridge) Default() {
	ros2bridgelog.Info("default", "name", r.Name)
}

//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha2-ros2bridge,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=ros2bridges,verbs=create;update,versions=v1alpha2,name=vros2bridge.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &ROS2Bridge{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *ROS2Bridge) ValidateCreate() error {
	ros2bridgelog.Info("validate create", "name", r.Name)
	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *ROS2Bridge) ValidateUpdate(old runtime.Object) error {
	ros2bridgelog.Info("validate update", "name", r.Name)
	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *ROS2Bridge) ValidateDelete() error {
	ros2bridgelog.Info("validate delete", "name", r.Name)
	return nil
}
