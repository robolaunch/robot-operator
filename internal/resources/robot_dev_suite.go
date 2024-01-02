package resources

import (
	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

func GetRobotVDI(robotDevSuite *robotv1alpha1.RobotDevSuite, robotVDINamespacedName *types.NamespacedName) *robotv1alpha1.RobotVDI {

	robotVDI := robotv1alpha1.RobotVDI{
		ObjectMeta: metav1.ObjectMeta{
			Name:      robotVDINamespacedName.Name,
			Namespace: robotVDINamespacedName.Namespace,
			Labels:    robotDevSuite.Labels,
		},
		Spec: robotDevSuite.Spec.RobotVDITemplate,
	}

	return &robotVDI
}

func GetRobotIDE(robotDevSuite *robotv1alpha1.RobotDevSuite, robotIDENamespacedName *types.NamespacedName) *robotv1alpha1.RobotIDE {

	robotIDE := robotv1alpha1.RobotIDE{
		ObjectMeta: metav1.ObjectMeta{
			Name:      robotIDENamespacedName.Name,
			Namespace: robotIDENamespacedName.Namespace,
			Labels:    robotDevSuite.Labels,
		},
		Spec: robotDevSuite.Spec.RobotIDETemplate,
	}

	if robotDevSuite.Spec.VDIEnabled {
		robotIDE.Labels[internal.TARGET_VDI_LABEL_KEY] = robotDevSuite.GetRobotVDIMetadata().Name
	}

	return &robotIDE
}

func GetNotebook(robotDevSuite *robotv1alpha1.RobotDevSuite, notebookNamespacedName *types.NamespacedName) *robotv1alpha1.Notebook {

	notebook := robotv1alpha1.Notebook{
		ObjectMeta: metav1.ObjectMeta{
			Name:      notebookNamespacedName.Name,
			Namespace: notebookNamespacedName.Namespace,
			Labels:    robotDevSuite.Labels,
		},
		Spec: robotDevSuite.Spec.NotebookTemplate,
	}

	if robotDevSuite.Spec.VDIEnabled {
		notebook.Labels[internal.TARGET_VDI_LABEL_KEY] = robotDevSuite.GetRobotVDIMetadata().Name
	}

	return &notebook
}

func GetRemoteIDERelayServer(robotDevSuite *robotv1alpha1.RobotDevSuite, relayServerNamespacedName *types.NamespacedName) *robotv1alpha1.RelayServer {

	relayServer := robotv1alpha1.RelayServer{
		ObjectMeta: metav1.ObjectMeta{
			Name:      relayServerNamespacedName.Name,
			Namespace: relayServerNamespacedName.Namespace,
			Labels:    robotDevSuite.Labels,
		},
		Spec: robotDevSuite.Spec.RemoteIDERelayServerTemplate,
	}

	return &relayServer
}
