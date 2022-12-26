package metrics_collector

import (
	"bytes"
	"context"
	goErr "errors"
	"fmt"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/tools/remotecommand"
)

func (r *MetricsCollectorReconciler) reconcileGetMetrics(ctx context.Context, instance *robotv1alpha1.MetricsCollector) error {
	var cmdBuilder strings.Builder
	// Get CPU usage
	cmdBuilder.WriteString("cat /sys/fs/cgroup/cpu/cpuacct.usage")
	cmdBuilder.WriteString(" && ")
	// Get Memory usage
	cmdBuilder.WriteString("cat /sys/fs/cgroup/memory/memory.usage_in_bytes")
	cmdBuilder.WriteString(" && ")
	// Get Network usage
	cmdBuilder.WriteString("cat /proc/net/dev | awk -F ' ' '{print $1 $2 \":\" $10}' | tail -n+3")

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
				err := updateCPUUsage(instance, &c.CPUUtilization, cpuData)
				if err != nil {
					c.CPUUtilization.Message = err.Error()
				}
			}

			if instance.Spec.Memory {
				err = updateMemoryUsage(instance, &c.MemoryUtilization, memoryData)
				if err != nil {
					c.MemoryUtilization.Message = err.Error()
				}
			}

			if instance.Spec.NetworkLoad {
				err = updateNetworkUsage(instance, &c.NetworkLoadUtilization, networkLoadData)
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

func updateCPUUsage(instance *robotv1alpha1.MetricsCollector, cpuUtil *robotv1alpha1.CPUUtilization, output string) error {

	var cpuUsage float64 = 0.0
	if cpuUtil.Value != "" {
		oldCpuUsage, err := strconv.ParseFloat(cpuUtil.Value, 64)
		if err != nil {
			return err
		}
		cpuUsage = oldCpuUsage
	}

	outputFloat64, err := strconv.ParseFloat(output, 64)
	if err != nil {
		return err
	}

	elapsedTimeNano := float64(time.Now().UnixNano() - instance.Status.LastUpdateTimestamp.UnixNano())
	corePercentage := fmt.Sprintf("%.2f", (outputFloat64-cpuUsage)*100/elapsedTimeNano) + "%"
	hostPercentage := fmt.Sprintf("%.2f", (outputFloat64-cpuUsage)*100/float64(instance.Status.Allocatable.Cpu().Value())/elapsedTimeNano) + "%"

	cpuUtil.Value = output
	cpuUtil.CorePercentage = corePercentage
	cpuUtil.HostPercentage = hostPercentage
	cpuUtil.Message = "active"

	return nil
}

func updateMemoryUsage(instance *robotv1alpha1.MetricsCollector, memUtil *robotv1alpha1.MemoryUtilization, output string) error {

	outputFloat64, err := strconv.ParseFloat(output, 64)
	if err != nil {
		return err
	}

	hostPercentage := fmt.Sprintf("%.2f", outputFloat64*100/instance.Status.Allocatable.Memory().AsApproximateFloat64()) + "%"

	memUtil.Value = output
	memUtil.HostPercentage = hostPercentage
	memUtil.Message = "active"

	return nil
}

func updateNetworkUsage(instance *robotv1alpha1.MetricsCollector, netUtil *robotv1alpha1.NetworkLoadUtilization, output []string) error {

	for _, niStr := range output {

		niData := strings.Split(niStr, ":")
		if len(niData) != 3 {
			return goErr.New("broken network data: " + niStr)
		}

		interfaceName := niData[0]
		interfaceReceive := niData[1]
		interfaceTransmit := niData[2]

		updated := false
		for k, iface := range netUtil.Interfaces {
			if interfaceName == iface.Name {
				err := updateNetworkInterfaceStatus(instance, &iface, interfaceReceive, interfaceTransmit)
				if err != nil {
					return err
				}
				netUtil.Interfaces[k] = iface
				updated = true
				break
			}
		}

		if !updated {
			netUtil.Interfaces = append(netUtil.Interfaces, robotv1alpha1.NetworkInterfaceUtilization{
				Name: interfaceName,
				Receive: robotv1alpha1.NetworkLoad{
					Value: interfaceReceive,
				},
				Transmit: robotv1alpha1.NetworkLoad{
					Value: interfaceTransmit,
				},
			})
		}

	}

	return nil
}

func updateNetworkInterfaceStatus(instance *robotv1alpha1.MetricsCollector, netInterface *robotv1alpha1.NetworkInterfaceUtilization, receive string, transmit string) error {

	elapsedTimeSeconds := float64(time.Now().Sub(instance.Status.LastUpdateTimestamp.Time).Seconds())

	oldReceiveFloat, err := strconv.ParseFloat(netInterface.Receive.Value, 64)
	if err != nil {
		return err
	}

	receiveFloat, err := strconv.ParseFloat(receive, 64)
	if err != nil {
		return err
	}

	oldTransmitFloat, err := strconv.ParseFloat(netInterface.Transmit.Value, 64)
	if err != nil {
		return err
	}

	transmitFloat, err := strconv.ParseFloat(transmit, 64)
	if err != nil {
		return err
	}

	netInterface.Receive.Value = fmt.Sprintf("%.2f", receiveFloat)
	netInterface.Transmit.Value = fmt.Sprintf("%.2f", transmitFloat)

	netInterface.Receive.Load = calculateUnit(receiveFloat, oldReceiveFloat, elapsedTimeSeconds)
	netInterface.Transmit.Load = calculateUnit(transmitFloat, oldTransmitFloat, elapsedTimeSeconds)

	netInterface.Receive.LoadKBit = calculateKBit(receiveFloat, oldReceiveFloat, elapsedTimeSeconds)
	netInterface.Transmit.LoadKBit = calculateKBit(transmitFloat, oldTransmitFloat, elapsedTimeSeconds)

	return nil
}

func calculateUnit(newVal float64, oldVal float64, elapsedTime float64) string {
	loadBytes := (newVal - oldVal) / elapsedTime

	var absoluteVal float64 = loadBytes
	var unit string = "Bytes/s"

	if int(loadBytes)%125 > 1 {

		if int(loadBytes)%125000 > 1 {

			if int(loadBytes)%125000000 > 1 {

				absoluteVal = loadBytes / 125000000
				unit = "gBits/s"

			} else {

				absoluteVal = loadBytes / 125000
				unit = "mBits/s"

			}

		} else {

			absoluteVal = loadBytes / 125
			unit = "kBits/s"

		}

	}

	return fmt.Sprintf("%.2f", absoluteVal) + " " + unit
}

func calculateKBit(newVal float64, oldVal float64, elapsedTime float64) string {
	loadBytes := (newVal - oldVal) / elapsedTime

	absoluteVal := loadBytes / 125
	unit := "kBits/s"

	return fmt.Sprintf("%.2f", absoluteVal) + " " + unit
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
