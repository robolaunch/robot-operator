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
var launchmanagerlog = logf.Log.WithName("launchmanager-resource")

func (r *LaunchManager) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

// TODO(user): EDIT THIS FILE!  THIS IS SCAFFOLDING FOR YOU TO OWN!

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha1-launchmanager,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=launchmanagers,verbs=create;update,versions=v1alpha1,name=mlaunchmanager.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &LaunchManager{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *LaunchManager) Default() {
	launchmanagerlog.Info("default", "name", r.Name)

	// TODO(user): fill in your defaulting logic.
}

// TODO(user): change verbs to "verbs=create;update;delete" if you want to enable deletion validation.
//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha1-launchmanager,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=launchmanagers,verbs=create;update,versions=v1alpha1,name=vlaunchmanager.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &LaunchManager{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *LaunchManager) ValidateCreate() error {
	launchmanagerlog.Info("validate create", "name", r.Name)

	err := r.checkTargetRobotLabel()
	if err != nil {
		return err
	}

	err = r.checkTargetRobotVDILabel()
	if err != nil {
		return err
	}

	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *LaunchManager) ValidateUpdate(old runtime.Object) error {
	launchmanagerlog.Info("validate update", "name", r.Name)

	err := r.checkTargetRobotLabel()
	if err != nil {
		return err
	}

	err = r.checkTargetRobotVDILabel()
	if err != nil {
		return err
	}

	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *LaunchManager) ValidateDelete() error {
	launchmanagerlog.Info("validate delete", "name", r.Name)
	return nil
}

func (r *LaunchManager) checkTargetRobotLabel() error {
	labels := r.GetLabels()

	if _, ok := labels[internal.TARGET_ROBOT]; !ok {
		return errors.New("target robot label should be added with key " + internal.TARGET_ROBOT)
	}

	return nil
}

func (r *LaunchManager) checkTargetRobotVDILabel() error {
	labels := r.GetLabels()

	if _, ok := labels[internal.TARGET_VDI]; !ok {
		return errors.New("target robot vdi label should be added with key " + internal.TARGET_VDI)
	}

	return nil
}
