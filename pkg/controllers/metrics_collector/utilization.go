package metrics_collector

import (
	"bytes"
	"context"
	"os"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/metrics"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/tools/remotecommand"
)

func (r *MetricsCollectorReconciler) reconcileGetMetrics(ctx context.Context, instance *robotv1alpha1.MetricsCollector) error {
	var cmdBuilder strings.Builder
	// Get CPU usage
	cmdBuilder.WriteString(internal.CMD_GET_CPU)
	cmdBuilder.WriteString(" && ")
	// Get Memory usage
	cmdBuilder.WriteString(internal.CMD_GET_MEMORY)
	cmdBuilder.WriteString(" && ")
	// Get Network usage
	cmdBuilder.WriteString(internal.CMD_GET_NETWORK_LOAD)

	for k, c := range instance.Status.ComponentMetrics {
		out, err := r.readValueFromContainer(instance, c.PodReference.Name, c.ContainerName, cmdBuilder.String())
		if err != nil {
			c.Message = err.Error()
		} else {

			data := strings.Split(out, "\n")
			if len(data) < 3 {
				c.Message = "broken data"
				instance.Status.ComponentMetrics[k] = c
				return nil
			}

			cpuData := data[0]
			memoryData := data[1]
			networkLoadData := data[2:]

			if instance.Spec.CPU {
				err := metrics.UpdateCPUUsage(instance, &c.CPUUtilization, cpuData)
				if err != nil {
					c.CPUUtilization.Message = err.Error()
				}
			}

			if instance.Spec.Memory {
				err = metrics.UpdateMemoryUsage(instance, &c.MemoryUtilization, memoryData)
				if err != nil {
					c.MemoryUtilization.Message = err.Error()
				}
			}

			if instance.Spec.NetworkLoad {
				err = metrics.UpdateNetworkUsage(instance, &c.NetworkLoadUtilization, networkLoadData)
				if err != nil {
					c.NetworkLoadUtilization.Message = err.Error()
				}
			}

			c.Message = "active"

			instance.Status.ComponentMetrics[k] = c
		}
	}

	return nil
}

func (r *MetricsCollectorReconciler) readValueFromContainer(instance *robotv1alpha1.MetricsCollector, podName string, containerName string, cmd string) (string, error) {

	inspectCommand := r.RESTClient.
		Post().
		Namespace(instance.Namespace).
		Resource("pods").
		Name(podName).
		SubResource("exec").
		VersionedParams(&corev1.PodExecOptions{
			Container: containerName,
			Command:   internal.Bash(cmd),
			Stdin:     true,
			Stdout:    true,
			Stderr:    true,
		}, runtime.NewParameterCodec(r.Scheme))

	exec, err := remotecommand.NewSPDYExecutor(r.RESTConfig, "POST", inspectCommand.URL())
	if err != nil {
		return "", err
	}

	var buff bytes.Buffer
	err = exec.Stream(remotecommand.StreamOptions{
		Stdin:  os.Stdin,
		Stdout: &buff,
		Stderr: os.Stderr,
		Tty:    false,
	})

	if err != nil {
		return "", err
	}

	output := string(buff.Bytes()[:])
	index := strings.LastIndex(output, "\n")
	output = output[:index] + strings.Replace(output[index:], "\n", "", 1)

	return output, nil
}
