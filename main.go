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

package main

import (
	"flag"
	"os"

	// Import all Kubernetes client auth plugins (e.g. Azure, GCP, OIDC, etc.)
	// to ensure that exec-entrypoint and run can make use of them.
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/kubernetes"
	_ "k8s.io/client-go/plugin/pkg/client/auth"

	"k8s.io/apimachinery/pkg/runtime"
	utilruntime "k8s.io/apimachinery/pkg/util/runtime"
	clientgoscheme "k8s.io/client-go/kubernetes/scheme"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/healthz"
	"sigs.k8s.io/controller-runtime/pkg/log/zap"
	"sigs.k8s.io/controller-runtime/pkg/manager"

	mcsv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/external/apis/mcsv1alpha1/v1alpha1"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	buildManager "github.com/robolaunch/robot-operator/pkg/controllers/build_manager"
	launchManager "github.com/robolaunch/robot-operator/pkg/controllers/launch_manager"
	"github.com/robolaunch/robot-operator/pkg/controllers/metrics"
	robot "github.com/robolaunch/robot-operator/pkg/controllers/robot"
	discoveryServer "github.com/robolaunch/robot-operator/pkg/controllers/robot/discovery_server"
	relayServer "github.com/robolaunch/robot-operator/pkg/controllers/robot/relay_server"
	rosBridge "github.com/robolaunch/robot-operator/pkg/controllers/robot/ros_bridge"
	robotDevSuite "github.com/robolaunch/robot-operator/pkg/controllers/robot_dev_suite"
	"github.com/robolaunch/robot-operator/pkg/controllers/robot_dev_suite/notebook"
	robotIDE "github.com/robolaunch/robot-operator/pkg/controllers/robot_dev_suite/robot_ide"
	robotVDI "github.com/robolaunch/robot-operator/pkg/controllers/robot_dev_suite/robot_vdi"
	ros2Workload "github.com/robolaunch/robot-operator/pkg/controllers/v1alpha2/ros2_workload"
	workspaceManager "github.com/robolaunch/robot-operator/pkg/controllers/workspace_manager"
	//+kubebuilder:scaffold:imports
)

var (
	scheme   = runtime.NewScheme()
	setupLog = ctrl.Log.WithName("setup")
)

func init() {
	utilruntime.Must(clientgoscheme.AddToScheme(scheme))

	utilruntime.Must(robotv1alpha1.AddToScheme(scheme))

	utilruntime.Must(mcsv1alpha1.AddToScheme(scheme))
	utilruntime.Must(robotv1alpha2.AddToScheme(scheme))
	//+kubebuilder:scaffold:scheme
}

func main() {
	var metricsAddr string
	var enableLeaderElection bool
	var probeAddr string
	flag.StringVar(&metricsAddr, "metrics-bind-address", ":8080", "The address the metric endpoint binds to.")
	flag.StringVar(&probeAddr, "health-probe-bind-address", ":8081", "The address the probe endpoint binds to.")
	flag.BoolVar(&enableLeaderElection, "leader-elect", false,
		"Enable leader election for controller manager. "+
			"Enabling this will ensure there is only one active controller manager.")
	opts := zap.Options{
		Development: true,
	}
	opts.BindFlags(flag.CommandLine)
	flag.Parse()

	ctrl.SetLogger(zap.New(zap.UseFlagOptions(&opts)))

	mgr, err := ctrl.NewManager(ctrl.GetConfigOrDie(), ctrl.Options{
		Scheme:                 scheme,
		MetricsBindAddress:     metricsAddr,
		Port:                   9443,
		HealthProbeBindAddress: probeAddr,
		LeaderElection:         enableLeaderElection,
		LeaderElectionID:       "ffc51351.roboscale.io",
		// LeaderElectionReleaseOnCancel defines if the leader should step down voluntarily
		// when the Manager ends. This requires the binary to immediately end when the
		// Manager is stopped, otherwise, this setting is unsafe. Setting this significantly
		// speeds up voluntary leader transitions as the new leader don't have to wait
		// LeaseDuration time first.
		//
		// In the default scaffold provided, the program ends immediately after
		// the manager stops, so would be fine to enable this option. However,
		// if you are doing or is intended to do any operation such as perform cleanups
		// after the manager stops then its usage might be unsafe.
		// LeaderElectionReleaseOnCancel: true,
	})
	if err != nil {
		setupLog.Error(err, "unable to start manager")
		os.Exit(1)
	}

	dynamicClient, err := dynamic.NewForConfig(mgr.GetConfig())
	if err != nil {
		setupLog.Error(err, "unable to create dynamic client")
	}

	clientset, err := kubernetes.NewForConfig(mgr.GetConfig())
	if err != nil {
		setupLog.Error(err, "unable to create clientset")
	}

	// Start controllers and webhooks
	// v1alpha1
	startRobotCRDsAndWebhooks(mgr, dynamicClient, *clientset)
	startManagerCRDsAndWebhooks(mgr, dynamicClient)
	startDevCRDsAndWebhooks(mgr, dynamicClient)
	startObserverCRDsAndWebhooks(mgr, dynamicClient)
	// v1alpha2
	startProductionCRDsAndWebhooks(mgr, dynamicClient)

	//+kubebuilder:scaffold:builder

	if err := mgr.AddHealthzCheck("healthz", healthz.Ping); err != nil {
		setupLog.Error(err, "unable to set up health check")
		os.Exit(1)
	}
	if err := mgr.AddReadyzCheck("readyz", healthz.Ping); err != nil {
		setupLog.Error(err, "unable to set up ready check")
		os.Exit(1)
	}

	setupLog.Info("starting manager")
	if err := mgr.Start(ctrl.SetupSignalHandler()); err != nil {
		setupLog.Error(err, "problem running manager")
		os.Exit(1)
	}
}

// This function starts Robot CRDs' controllers and webhooks. Here are the CRDs:
// - Robot (robots.robot.roboscale.io/v1alpha1)
// - DiscoveryServer (discoveryservers.robot.roboscale.io/v1alpha1)
// - ROSBridge (rosbridges.robot.roboscale.io/v1alpha1)
// - RelayServer (relayservers.robot.roboscale.io/v1alpha1)
func startRobotCRDsAndWebhooks(mgr manager.Manager, dynamicClient dynamic.Interface, clientset kubernetes.Clientset) {

	// Start Robot controller & webhook
	if err := (&robot.RobotReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
		Clientset:     clientset,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "Robot")
		os.Exit(1)
	}
	if err := (&robotv1alpha1.Robot{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "Robot")
		os.Exit(1)
	}

	// Start DiscoveryServer controller & webhook
	if err := (&discoveryServer.DiscoveryServerReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "DiscoveryServer")
		os.Exit(1)
	}
	if err := (&robotv1alpha1.DiscoveryServer{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "DiscoveryServer")
		os.Exit(1)
	}

	// Start ROSBridge controller
	if err := (&rosBridge.ROSBridgeReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "ROSBridge")
		os.Exit(1)
	}

	// Start RelayServer controller
	if err := (&relayServer.RelayServerReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "RelayServer")
		os.Exit(1)
	}

}

// This function starts Manager CRDs' controllers and webhooks. Here are the CRDs:
// - WorkspaceManager (workspacemanagers.robot.roboscale.io/v1alpha1)
// - BuildManager (buildmanagers.robot.roboscale.io/v1alpha1)
// - LaunchManager (launchmanagers.robot.roboscale.io/v1alpha1)
func startManagerCRDsAndWebhooks(mgr manager.Manager, dynamicClient dynamic.Interface) {

	// Start WorkspaceManager controller & webhooks
	if err := (&workspaceManager.WorkspaceManagerReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "WorkspaceManager")
		os.Exit(1)
	}
	if err := (&robotv1alpha1.WorkspaceManager{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "WorkspaceManager")
		os.Exit(1)
	}

	// Start BuildManager controller & webhooks
	if err := (&buildManager.BuildManagerReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "BuildManager")
		os.Exit(1)
	}
	if err := (&robotv1alpha1.BuildManager{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "BuildManager")
		os.Exit(1)
	}

	// Start LaunchManager controller & webhooks
	if err := (&launchManager.LaunchManagerReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "LaunchManager")
		os.Exit(1)
	}
	if err := (&robotv1alpha1.LaunchManager{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "LaunchManager")
		os.Exit(1)
	}
}

// This function starts Dev CRDs' controllers and webhooks. Here are the CRDs:
// - RobotDevSuite (robotdevsuites.robot.roboscale.io/v1alpha1)
// - RobotIDE (robotides.robot.roboscale.io/v1alpha1)
// - RobotVDI (robotvdis.robot.roboscale.io/v1alpha1)
func startDevCRDsAndWebhooks(mgr manager.Manager, dynamicClient dynamic.Interface) {

	// Start RobotDevSuite controller & webhooks
	if err := (&robotDevSuite.RobotDevSuiteReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "RobotDevSuite")
		os.Exit(1)
	}

	// Start RobotVDI controller & webhooks
	if err := (&robotVDI.RobotVDIReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "RobotVDI")
		os.Exit(1)
	}
	if err := (&robotv1alpha1.RobotVDI{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "RobotVDI")
		os.Exit(1)
	}

	// Start RobotIDE controller & webhooks
	if err := (&robotIDE.RobotIDEReconciler{
		Client:        mgr.GetClient(),
		Scheme:        mgr.GetScheme(),
		DynamicClient: dynamicClient,
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "RobotIDE")
		os.Exit(1)
	}
	if err := (&robotv1alpha1.RobotIDE{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "RobotIDE")
		os.Exit(1)
	}

	if err := (&notebook.NotebookReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "Notebook")
		os.Exit(1)
	}
	if err := (&robotv1alpha1.Notebook{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "Notebook")
		os.Exit(1)
	}
}

// This function starts Observer CRDs' controllers and webhooks. Here are the CRDs:
// - MetricsExporter (metricsexporters.robot.roboscale.io/v1alpha1)
func startObserverCRDsAndWebhooks(mgr manager.Manager, dynamicClient dynamic.Interface) {

	// Start MetricsExporter controller & webhooks
	if err := (&metrics.MetricsExporterReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "MetricsExporter")
		os.Exit(1)
	}
	if err := (&robotv1alpha1.RobotDevSuite{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "RobotDevSuite")
		os.Exit(1)
	}
}

// This function starts Production CRDs' controllers and webhooks. Here are the CRDs:
// - ROS2Workload (ros2workloads.robot.roboscale.io/v1alpha2)
func startProductionCRDsAndWebhooks(mgr manager.Manager, dynamicClient dynamic.Interface) {
	if err := (&ros2Workload.ROS2WorkloadReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "ROS2Workload")
		os.Exit(1)
	}
	if err := (&robotv1alpha2.ROS2Workload{}).SetupWebhookWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create webhook", "webhook", "ROS2Workload")
		os.Exit(1)
	}
}
