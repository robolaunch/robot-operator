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

// log is for logging in this package.
var buildmanagerlog = logf.Log.WithName("buildmanager-resource")

func (r *BuildManager) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha1-buildmanager,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=buildmanagers,verbs=create;update,versions=v1alpha1,name=mbuildmanager.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &BuildManager{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *BuildManager) Default() {
	buildmanagerlog.Info("default", "name", r.Name)
}

//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha1-buildmanager,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=buildmanagers,verbs=create;update,versions=v1alpha1,name=vbuildmanager.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &BuildManager{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *BuildManager) ValidateCreate() error {
	buildmanagerlog.Info("validate create", "name", r.Name)

	err := r.checkTargetRobotLabel()
	if err != nil {
		return err
	}

	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *BuildManager) ValidateUpdate(old runtime.Object) error {
	buildmanagerlog.Info("validate update", "name", r.Name)

	err := r.checkTargetRobotLabel()
	if err != nil {
		return err
	}

	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *BuildManager) ValidateDelete() error {
	buildmanagerlog.Info("validate delete", "name", r.Name)
	return nil
}

func (r *BuildManager) checkTargetRobotLabel() error {
	labels := r.GetLabels()

	if _, ok := labels[internal.TARGET_ROBOT_LABEL_KEY]; !ok {
		return errors.New("target robot label should be added with key " + internal.TARGET_ROBOT_LABEL_KEY)
	}

	return nil
}
