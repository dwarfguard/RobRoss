# Realtime Host Policy

Install the checked-in limits file, add the robot operator to the `realtime`
group, then log out and back in (or reboot):

```bash
sudo groupadd --force realtime
sudo usermod -aG realtime "$USER"
sudo install -o root -g root -m 0644 \
  deployment/realtime/99-robross-realtime.conf \
  /etc/security/limits.d/99-robross-realtime.conf
```

Verify `ulimit -r` reports at least `50`. After starting the physical driver,
verify the `aubo_ros2_control_node` control thread is `SCHED_FIFO/50`; the host
preflight checks permission before startup, and qualification evidence records
the active policy.
