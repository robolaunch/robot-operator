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
	"errors"

	"github.com/robolaunch/robot-operator/internal"
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	logf "sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
)

// ********************************
// MetricsCollector webhooks
// ********************************

// log is for logging in this package.
var metricscollectorlog = logf.Log.WithName("metricscollector-resource")

func (r *MetricsCollector) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha1-metricscollector,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=metricscollectors,verbs=create;update,versions=v1alpha1,name=mmetricscollector.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &MetricsCollector{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *MetricsCollector) Default() {
	metricscollectorlog.Info("default", "name", r.Name)
}

//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha1-metricscollector,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=metricscollectors,verbs=create;update,versions=v1alpha1,name=vmetricscollector.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &MetricsCollector{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *MetricsCollector) ValidateCreate() error {
	metricscollectorlog.Info("validate create", "name", r.Name)

	err := r.checkTargetRobotLabel()
	if err != nil {
		return err
	}

	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *MetricsCollector) ValidateUpdate(old runtime.Object) error {
	metricscollectorlog.Info("validate update", "name", r.Name)

	err := r.checkTargetRobotLabel()
	if err != nil {
		return err
	}

	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *MetricsCollector) ValidateDelete() error {
	metricscollectorlog.Info("validate delete", "name", r.Name)
	return nil
}

func (r *MetricsCollector) checkTargetRobotLabel() error {
	labels := r.GetLabels()

	if _, ok := labels[internal.TARGET_ROBOT_LABEL_KEY]; !ok {
		return errors.New("target robot label should be added with key " + internal.TARGET_ROBOT_LABEL_KEY)
	}

	return nil
}
