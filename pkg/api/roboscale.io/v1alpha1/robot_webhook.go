package v1alpha1

import (
	"errors"
	"reflect"
	"regexp"

	"github.com/robolaunch/robot-operator/internal"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	logf "sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
)

// log is for logging in this package.
var robotlog = logf.Log.WithName("robot-resource")

func (r *Robot) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha1-robot,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=robots,verbs=create;update,versions=v1alpha1,name=mrobot.kb.io,admissionReviewVersions=v1

// ********************************
// Robot webhooks
// ********************************

var _ webhook.Defaulter = &Robot{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *Robot) Default() {
	robotlog.Info("default", "name", r.Name)

	r.setDefaultLabels()
	r.setRepositoryPaths()
	_ = r.setRepositoryInfo()
	r.setWorkspacesPath()
	r.setDiscoveryServerDomainID()
	r.setDiscoveryServerProtocol()
}

//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha1-robot,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=robots,verbs=create;update,versions=v1alpha1,name=vrobot.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &Robot{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *Robot) ValidateCreate() error {
	robotlog.Info("validate create", "name", r.Name)

	err := r.checkTenancyLabels()
	if err != nil {
		return err
	}

	err = r.checkRobotType()
	if err != nil {
		return err
	}

	err = r.checkWorkspaces()
	if err != nil {
		return err
	}

	err = r.checkRobotDevSuite()
	if err != nil {
		return err
	}

	err = r.checkDistributions()
	if err != nil {
		return err
	}

	err = r.checkAdditionalConfigs()
	if err != nil {
		return err
	}

	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *Robot) ValidateUpdate(old runtime.Object) error {
	robotlog.Info("validate update", "name", r.Name)

	err := r.checkTenancyLabels()
	if err != nil {
		return err
	}

	err = r.checkRobotType()
	if err != nil {
		return err
	}

	err = r.checkWorkspaces()
	if err != nil {
		return err
	}

	err = r.checkRobotDevSuite()
	if err != nil {
		return err
	}

	err = r.checkDistributions()
	if err != nil {
		return err
	}

	err = r.checkAdditionalConfigs()
	if err != nil {
		return err
	}

	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *Robot) ValidateDelete() error {
	robotlog.Info("validate delete", "name", r.Name)
	return nil
}

func (r *Robot) checkRobotType() error {

	if reflect.DeepEqual(r.Spec.Type, TypeRobot) {
		if reflect.DeepEqual(r.Spec.RobotConfig, nil) || reflect.DeepEqual(r.Spec.RobotConfig, RobotConfig{}) {
			return errors.New("`spec.robot` cannot be empty if the robot type is Robot")
		}
		if !(reflect.DeepEqual(r.Spec.EnvironmentConfig, nil) || reflect.DeepEqual(r.Spec.EnvironmentConfig, EnvironmentConfig{})) {
			return errors.New("`spec.environment` should be empty if the robot type is Robot")
		}
	}

	if reflect.DeepEqual(r.Spec.Type, TypeEnvironment) {
		if reflect.DeepEqual(r.Spec.EnvironmentConfig, nil) || reflect.DeepEqual(r.Spec.EnvironmentConfig, EnvironmentConfig{}) {
			return errors.New("`spec.environment` cannot be empty if the robot type is Environment")
		}
		if !(reflect.DeepEqual(r.Spec.RobotConfig, nil) || reflect.DeepEqual(r.Spec.RobotConfig, RobotConfig{})) {
			return errors.New("`spec.robot` should be empty if the robot type is Environment")
		}
	}

	return nil

}

func (r *Robot) checkTenancyLabels() error {
	labels := r.GetLabels()

	if _, ok := labels[internal.ORGANIZATION_LABEL_KEY]; !ok {
		return errors.New("organization label should be added with key " + internal.ORGANIZATION_LABEL_KEY)
	}

	if _, ok := labels[internal.TEAM_LABEL_KEY]; !ok {
		return errors.New("team label should be added with key " + internal.TEAM_LABEL_KEY)
	}

	if _, ok := labels[internal.REGION_LABEL_KEY]; !ok {
		return errors.New("super cluster label should be added with key " + internal.REGION_LABEL_KEY)
	}

	if _, ok := labels[internal.CLOUD_INSTANCE_LABEL_KEY]; !ok {
		return errors.New("cloud instance label should be added with key " + internal.CLOUD_INSTANCE_LABEL_KEY)
	}

	if _, ok := labels[internal.CLOUD_INSTANCE_ALIAS_LABEL_KEY]; !ok {
		return errors.New("cloud instance alias label should be added with key " + internal.CLOUD_INSTANCE_ALIAS_LABEL_KEY)
	}
	return nil
}

func (r *Robot) checkDistributions() error {
	if reflect.DeepEqual(r.Spec.Type, TypeRobot) {
		if len(r.Spec.RobotConfig.Distributions) == 2 && (r.Spec.RobotConfig.Distributions[0] == ROSDistroHumble || r.Spec.RobotConfig.Distributions[1] == ROSDistroHumble) {
			return errors.New("humble cannot be used in a multidistro environment")
		}
	}

	return nil
}

func (r *Robot) checkWorkspaces() error {

	if reflect.DeepEqual(r.Spec.Type, TypeRobot) {
		for _, ws := range r.Spec.WorkspaceManagerTemplate.Workspaces {

			distroExists := false
			for _, distro := range r.Spec.RobotConfig.Distributions {
				if ws.Distro == distro {
					distroExists = true
					break
				}
			}

			if !distroExists {
				return errors.New("workspace " + ws.Name + " has unsupported distro defined in `spec.distributions`")
			}
		}
	}

	return nil
}

func (r *Robot) checkRobotDevSuite() error {

	dst := r.Spec.RobotDevSuiteTemplate

	if dst.IDEEnabled && dst.RobotIDETemplate.Display && !dst.VDIEnabled {
		return errors.New("cannot open an ide with a display when vdi disabled")
	}

	return nil
}

func (r *Robot) checkAdditionalConfigs() error {

	if val, ok := r.Spec.AdditionalConfigs[internal.IDE_CUSTOM_PORT_RANGE_KEY]; ok && val.ConfigType == AdditionalConfigTypeOperator {
		matched, err := regexp.MatchString(internal.CUSTOM_PORT_RANGE_REGEX, val.Value)
		if !matched {
			return errors.New("cannot validate ide ports, use this pattern " + internal.CUSTOM_PORT_RANGE_REGEX)
		}
		if err != nil {
			return err
		}
	}

	if val, ok := r.Spec.AdditionalConfigs[internal.VDI_CUSTOM_PORT_RANGE_KEY]; ok && val.ConfigType == AdditionalConfigTypeOperator {
		matched, err := regexp.MatchString(internal.CUSTOM_PORT_RANGE_REGEX, val.Value)
		if !matched {
			return errors.New("cannot validate ide ports, use this pattern " + internal.CUSTOM_PORT_RANGE_REGEX)
		}
		if err != nil {
			return err
		}
	}

	if val, ok := r.Spec.AdditionalConfigs[internal.PERSISTENT_DIRS_KEY]; ok && val.ConfigType == AdditionalConfigTypeOperator {
		matched, err := regexp.MatchString(internal.PERSISTENT_DIRS_REGEX, val.Value)
		if !matched {
			return errors.New("cannot set persistent directories, use this pattern " + internal.PERSISTENT_DIRS_REGEX)
		}
		if err != nil {
			return err
		}
	}

	if val, ok := r.Spec.AdditionalConfigs[internal.GRANT_PERMISSION_KEY]; ok && val.ConfigType == AdditionalConfigTypeOperator {
		matched, err := regexp.MatchString(internal.GRANT_PERMISSION_REGEX, val.Value)
		if !matched {
			return errors.New("cannot set persistent directories, use this pattern " + internal.GRANT_PERMISSION_REGEX)
		}
		if err != nil {
			return err
		}
	}

	if val, ok := r.Spec.AdditionalConfigs[internal.HOST_DIRS_KEY]; ok && val.ConfigType == AdditionalConfigTypeOperator {
		matched, err := regexp.MatchString(internal.HOST_DIRS_REGEX, val.Value)
		if !matched {
			return errors.New("cannot set host directories, use this pattern " + internal.HOST_DIRS_REGEX)
		}
		if err != nil {
			return err
		}
	}

	if val, ok := r.Spec.AdditionalConfigs[internal.SHARED_MEMORY_SIZE_KEY]; ok && val.ConfigType == AdditionalConfigTypeOperator {
		matched, err := regexp.MatchString(internal.SHARED_MEMORY_SIZE_REGEX, val.Value)
		if !matched {
			return errors.New("cannot set shared memory size, use this pattern " + internal.SHARED_MEMORY_SIZE_REGEX)
		}
		if err != nil {
			return err
		}
	}

	if val, ok := r.Spec.AdditionalConfigs[internal.HOST_NETWORK_SELECTION_KEY]; ok && val.ConfigType == AdditionalConfigTypeOperator {
		matched, err := regexp.MatchString(internal.HOST_NETWORK_SELECTION_REGEX, val.Value)
		if !matched {
			return errors.New("cannot set host network, use this pattern " + internal.HOST_NETWORK_SELECTION_REGEX)
		}
		if err != nil {
			return err
		}
	}

	return nil
}

func (r *Robot) setDefaultLabels() {
	if _, ok := r.Labels[internal.IMAGE_REGISTRY_LABEL_KEY]; !ok {
		r.Labels[internal.IMAGE_REGISTRY_LABEL_KEY] = "docker.io"
	}
}

func (r *Robot) setRepositoryPaths() {
	for wsKey := range r.Spec.WorkspaceManagerTemplate.Workspaces {
		ws := r.Spec.WorkspaceManagerTemplate.Workspaces[wsKey]
		for repoKey := range ws.Repositories {
			repo := ws.Repositories[repoKey]
			repo.Path = r.Spec.WorkspaceManagerTemplate.WorkspacesPath + "/" + ws.Name + "/src/" + repoKey
			ws.Repositories[repoKey] = repo
		}
		r.Spec.WorkspaceManagerTemplate.Workspaces[wsKey] = ws
	}
}

func (r *Robot) setRepositoryInfo() error {

	for k1, ws := range r.Spec.WorkspaceManagerTemplate.Workspaces {
		for k2, repo := range ws.Repositories {
			owner, repoName, err := getPathVariables(repo.URL)
			if err != nil {
				return err
			}

			repo.Owner = owner
			repo.Repo = repoName

			lastCommitHash, err := getLastCommitHash(repo)
			if err != nil {
				return err
			}

			repo.Hash = lastCommitHash

			ws.Repositories[k2] = repo
		}
		r.Spec.WorkspaceManagerTemplate.Workspaces[k1] = ws
	}

	return nil

}

func (r *Robot) setWorkspacesPath() {
	if reflect.DeepEqual(r.Spec.WorkspaceManagerTemplate, WorkspaceManagerSpec{}) {
		r.Spec.WorkspaceManagerTemplate = WorkspaceManagerSpec{
			WorkspacesPath: defaultWorkspacePath,
		}
	}

	if reflect.DeepEqual(r.Spec.WorkspaceManagerTemplate.WorkspacesPath, "") {
		r.Spec.WorkspaceManagerTemplate.WorkspacesPath = defaultWorkspacePath
	}
}

func (r *Robot) setDiscoveryServerDomainID() {
	if reflect.DeepEqual(r.Spec.Type, TypeRobot) {
		r.Spec.RobotConfig.DiscoveryServerTemplate.DomainID = r.Spec.RobotConfig.DomainID
	}
}

func (r *Robot) setDiscoveryServerProtocol() {
	if reflect.DeepEqual(r.Spec.Type, TypeRobot) {
		r.Spec.RobotConfig.DiscoveryServerTemplate.Protocol = "UDP"
	}
}

// ********************************
// DiscoveryServer webhooks
// ********************************

// log is for logging in this package.
var discoveryserverlog = logf.Log.WithName("discoveryserver-resource")

func (r *DiscoveryServer) SetupWebhookWithManager(mgr ctrl.Manager) error {
	return ctrl.NewWebhookManagedBy(mgr).
		For(r).
		Complete()
}

//+kubebuilder:webhook:path=/mutate-robot-roboscale-io-v1alpha1-discoveryserver,mutating=true,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=discoveryservers,verbs=create;update,versions=v1alpha1,name=mdiscoveryserver.kb.io,admissionReviewVersions=v1

var _ webhook.Defaulter = &DiscoveryServer{}

// Default implements webhook.Defaulter so a webhook will be registered for the type
func (r *DiscoveryServer) Default() {
	discoveryserverlog.Info("default", "name", r.Name)
}

//+kubebuilder:webhook:path=/validate-robot-roboscale-io-v1alpha1-discoveryserver,mutating=false,failurePolicy=fail,sideEffects=None,groups=robot.roboscale.io,resources=discoveryservers,verbs=create;update,versions=v1alpha1,name=vdiscoveryserver.kb.io,admissionReviewVersions=v1

var _ webhook.Validator = &DiscoveryServer{}

// ValidateCreate implements webhook.Validator so a webhook will be registered for the type
func (r *DiscoveryServer) ValidateCreate() error {
	discoveryserverlog.Info("validate create", "name", r.Name)

	err := r.checkContainerInfo()
	if err != nil {
		return err
	}

	return nil
}

// ValidateUpdate implements webhook.Validator so a webhook will be registered for the type
func (r *DiscoveryServer) ValidateUpdate(old runtime.Object) error {
	discoveryserverlog.Info("validate update", "name", r.Name)

	err := r.checkContainerInfo()
	if err != nil {
		return err
	}

	return nil
}

// ValidateDelete implements webhook.Validator so a webhook will be registered for the type
func (r *DiscoveryServer) ValidateDelete() error {
	discoveryserverlog.Info("validate delete", "name", r.Name)
	return nil
}

func (r *DiscoveryServer) checkContainerInfo() error {

	if r.Spec.Type == DiscoveryServerInstanceTypeServer {
		if !reflect.DeepEqual(r.Spec.Reference, corev1.ObjectReference{}) {
			return errors.New("reference should be nil if the type is server")
		}

		if !reflect.DeepEqual(r.Spec.Cluster, "") {
			return errors.New("cluster should be nil if the type is server")
		}
	} else if r.Spec.Type == DiscoveryServerInstanceTypeClient {
		if reflect.DeepEqual(r.Spec.Reference, corev1.ObjectReference{}) {
			return errors.New("reference cannot be nil if the type is client")
		}

		if reflect.DeepEqual(r.Spec.Cluster, "") {
			return errors.New("cluster cannot be nil if the type is client")
		}
	}

	return nil
}
