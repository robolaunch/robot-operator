package v1alpha1

import (
	"errors"
	"reflect"
	"regexp"

	"github.com/robolaunch/robot-operator/internal"
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	logf "sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
)

// ********************************
// Notebook webhooks
// ********************************

// log is for logging in this package.
var notebooklog = logf.Log.WithName("notebook-resource")

func (r *Notebook) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha1-notebook,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=notebooks,verbs=create;update,versions=v1alpha1,name=mnotebook.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &Notebook{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *Notebook) Default() {
	notebooklog.Info("default", "name", r.Name)
}

//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha1-notebook,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=notebooks,verbs=create;update,versions=v1alpha1,name=vnotebook.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &Notebook{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *Notebook) ValidateCreate() error {
	notebooklog.Info("validate create", "name", r.Name)

	err := r.checkPortLabels()
	if err != nil {
		return err
	}

	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *Notebook) ValidateUpdate(old runtime.Object) error {
	notebooklog.Info("validate update", "name", r.Name)

	err := r.checkPortLabels()
	if err != nil {
		return err
	}

	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *Notebook) ValidateDelete() error {
	notebooklog.Info("validate delete", "name", r.Name)
	return nil
}

func (r *Notebook) checkPortLabels() error {
	labels := r.GetLabels()

	if val, ok := labels[internal.NOTEBOOK_PORT_KEY]; !ok {
		matched, err := regexp.MatchString(internal.INTERNAL_APP_PORT_REGEX, val)
		if !matched {
			return errors.New("cannot set application port for Notebook, use this pattern " + internal.INTERNAL_APP_PORT_REGEX)
		}
		if err != nil {
			return err
		}
	}

	if val, ok := labels[internal.NOTEBOOK_FB_PORT_KEY]; !ok {
		matched, err := regexp.MatchString(internal.INTERNAL_APP_PORT_REGEX, val)
		if !matched {
			return errors.New("cannot set application port for file browser (Notebook), use this pattern " + internal.INTERNAL_APP_PORT_REGEX)
		}
		if err != nil {
			return err
		}
	}

	return nil
}

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

	err = r.checkPortLabels()
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

	err = r.checkPortLabels()
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

func (r *RobotIDE) checkPortLabels() error {
	labels := r.GetLabels()

	if val, ok := labels[internal.ROBOT_IDE_PORT_KEY]; !ok {
		matched, err := regexp.MatchString(internal.INTERNAL_APP_PORT_REGEX, val)
		if !matched {
			return errors.New("cannot set application port for IDE, use this pattern " + internal.INTERNAL_APP_PORT_REGEX)
		}
		if err != nil {
			return err
		}
	}

	if val, ok := labels[internal.ROBOT_IDE_FB_PORT_KEY]; !ok {
		matched, err := regexp.MatchString(internal.INTERNAL_APP_PORT_REGEX, val)
		if !matched {
			return errors.New("cannot set application port for file browser (IDE), use this pattern " + internal.INTERNAL_APP_PORT_REGEX)
		}
		if err != nil {
			return err
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

	err = r.checkPortLabels()
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

	err = r.checkPortLabels()
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

func (r *RobotVDI) checkPortLabels() error {
	labels := r.GetLabels()

	if val, ok := labels[internal.ROBOT_VDI_PORT_KEY]; !ok {
		matched, err := regexp.MatchString(internal.INTERNAL_APP_PORT_REGEX, val)
		if !matched {
			return errors.New("cannot set application port for VDI, use this pattern " + internal.INTERNAL_APP_PORT_REGEX)
		}
		if err != nil {
			return err
		}
	}

	if val, ok := labels[internal.ROBOT_VDI_FB_PORT_KEY]; !ok {
		matched, err := regexp.MatchString(internal.INTERNAL_APP_PORT_REGEX, val)
		if !matched {
			return errors.New("cannot set application port for file browser (VDI), use this pattern " + internal.INTERNAL_APP_PORT_REGEX)
		}
		if err != nil {
			return err
		}
	}

	return nil
}

// ********************************
// RobotDevSuite webhooks
// ********************************

// log is for logging in this package.
var robotdevsuitelog = logf.Log.WithName("robotdevsuite-resource")

func (r *RobotDevSuite) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha1-robotdevsuite,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=robotdevsuites,verbs=create;update,versions=v1alpha1,name=mrobotdevsuite.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &RobotDevSuite{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *RobotDevSuite) Default() {
	robotdevsuitelog.Info("default", "name", r.Name)
	DefaultRemoteIDERelayServerFields(r)
}

//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha1-robotdevsuite,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=robotdevsuites,verbs=create;update,versions=v1alpha1,name=vrobotdevsuite.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &RobotDevSuite{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *RobotDevSuite) ValidateCreate() error {
	robotdevsuitelog.Info("validate create", "name", r.Name)

	err := r.checkInstanceName()
	if err != nil {
		return err
	}

	err = r.checkRemoteIDERelayServerTemplate()
	if err != nil {
		return err
	}

	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *RobotDevSuite) ValidateUpdate(old runtime.Object) error {
	robotdevsuitelog.Info("validate update", "name", r.Name)

	err := r.checkInstanceName()
	if err != nil {
		return err
	}

	err = r.checkRemoteIDERelayServerTemplate()
	if err != nil {
		return err
	}

	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *RobotDevSuite) ValidateDelete() error {
	robotdevsuitelog.Info("validate delete", "name", r.Name)
	return nil
}

func (r *RobotDevSuite) checkInstanceName() error {
	if r.Spec.RemoteIDEEnabled {
		if r.Spec.RemoteIDERelayServerTemplate.InstanceName == "" {
			return errors.New("remote instance name for relay server cannot be empty")
		}
	}

	return nil
}

func (r *RobotDevSuite) checkRemoteIDERelayServerTemplate() error {

	if r.Spec.RemoteIDEEnabled {
		if reflect.DeepEqual(r.Spec.RemoteIDERelayServerTemplate, RelayServerSpec{}) {
			return errors.New("relay server template cannot be nil if remote ide relay server is enabled")
		}
	}

	return nil
}

func DefaultRemoteIDERelayServerFields(r *RobotDevSuite) {
	if r.Spec.RemoteIDEEnabled && !reflect.DeepEqual(r.Spec.RemoteIDERelayServerTemplate, RelayServerSpec{}) {
		instanceName := r.Spec.RemoteIDERelayServerTemplate.InstanceName
		rootDNSConfig := r.Spec.RemoteIDERelayServerTemplate.RootDNSConfig
		tlsSecretRef := r.Spec.RemoteIDERelayServerTemplate.TLSSecretReference

		r.Spec.RemoteIDERelayServerTemplate = RelayServerSpec{
			Hostname:           r.GetRobotIDEMetadata().Name,
			Subdomain:          r.GetRobotIDEMetadata().Name + internal.SVC_IDE_POSTFIX + "-" + instanceName,
			InstanceName:       instanceName,
			RemoteNamespace:    r.Namespace,
			RemotePort:         9000,
			RootDNSConfig:      rootDNSConfig,
			TLSSecretReference: tlsSecretRef,
		}
	}
}
