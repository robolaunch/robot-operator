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
// RobotIDE webhooks
// ********************************

// log is for logging in this package.
var robotidelog = logf.Log.WithName("robotide-resource")

func (r *RobotIDE) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha1-robotide,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=robotides,verbs=create;update,versions=v1alpha1,name=mrobotide.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &RobotIDE{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *RobotIDE) Default() {
	robotidelog.Info("default", "name", r.Name)
}

//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha1-robotide,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=robotides,verbs=create;update,versions=v1alpha1,name=vrobotide.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &RobotIDE{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *RobotIDE) ValidateCreate() error {
	robotidelog.Info("validate create", "name", r.Name)

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
func (r *RobotIDE) ValidateUpdate(old runtime.Object) error {
	robotidelog.Info("validate update", "name", r.Name)

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
func (r *RobotIDE) ValidateDelete() error {
	robotidelog.Info("validate delete", "name", r.Name)
	return nil
}

func (r *RobotIDE) checkTargetRobotLabel() error {
	labels := r.GetLabels()

	if _, ok := labels[internal.TARGET_ROBOT_LABEL_KEY]; !ok {
		return errors.New("target robot label should be added with key " + internal.TARGET_ROBOT_LABEL_KEY)
	}

	return nil
}

func (r *RobotIDE) checkTargetRobotVDILabel() error {
	labels := r.GetLabels()

	if r.Spec.Display {
		if _, ok := labels[internal.TARGET_VDI_LABEL_KEY]; !ok {
			return errors.New("target robot vdi label should be added with key " + internal.TARGET_VDI_LABEL_KEY)
		}
	}

	return nil
}

// ********************************
// RobotVDI webhooks
// ********************************

// log is for logging in this package.
var robotvdilog = logf.Log.WithName("robotvdi-resource")

func (r *RobotVDI) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha1-robotvdi,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=robotvdis,verbs=create;update,versions=v1alpha1,name=mrobotvdi.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &RobotVDI{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *RobotVDI) Default() {
	robotvdilog.Info("default", "name", r.Name)
}

//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha1-robotvdi,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=robotvdis,verbs=create;update,versions=v1alpha1,name=vrobotvdi.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &RobotVDI{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *RobotVDI) ValidateCreate() error {
	robotvdilog.Info("validate create", "name", r.Name)

	err := r.checkTargetRobotLabel()
	if err != nil {
		return err
	}

	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *RobotVDI) ValidateUpdate(old runtime.Object) error {
	robotvdilog.Info("validate update", "name", r.Name)

	err := r.checkTargetRobotLabel()
	if err != nil {
		return err
	}

	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *RobotVDI) ValidateDelete() error {
	robotvdilog.Info("validate delete", "name", r.Name)
	return nil
}

func (r *RobotVDI) checkTargetRobotLabel() error {
	labels := r.GetLabels()

	if _, ok := labels[internal.TARGET_ROBOT_LABEL_KEY]; !ok {
		return errors.New("target robot label should be added with key " + internal.TARGET_VDI_LABEL_KEY)
	}

	return nil
}
