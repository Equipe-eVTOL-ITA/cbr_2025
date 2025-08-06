"""
Independent network performance monitoring module.
Provides real network metrics (ping) and ROS2 topic health assessment.
"""

import time
import subprocess
import threading
from dataclasses import dataclass
from typing import Dict, Optional, List

@dataclass
class NetworkHealth:
    """Overall network health assessment."""
    overall_score: float  # 0-100
    status: str          # "excellent", "good", "fair", "poor", "critical"
    ping_latency: float  # ms
    packet_loss: float   # percentage
    ros2_health: float   # 0-100 based on topic performance

@dataclass
class TopicHealth:
    """Health status for a specific topic."""
    topic_name: str
    expected_rate: float
    actual_rate: float
    health_score: float  # 0-100
    last_message_time: float
    jitter: float        # rate variance

class NetworkMetrics:
    """Comprehensive network performance monitoring."""
    
    # Expected topic rates
    TOPIC_RATES = {
        '/telemetry/position': 20.0,      # 20Hz
        '/telemetry/drone_status': 2.0,   # 2Hz  
        '/telemetry/system_health': 1.0,  # 1Hz
    }
    
    def __init__(self, ping_target: str = "192.168.0.152", ground_station_ip: str = "192.168.0.100"):
        """Initialize network metrics with configurable ping targets."""
        self.ping_target = ping_target
        self.ground_station_ip = ground_station_ip
        self.topic_stats: Dict[str, List[float]] = {}
        self.topic_health: Dict[str, TopicHealth] = {}
        self._ping_results: List[float] = []
        self._running = False
        self._ping_thread: Optional[threading.Thread] = None
        
    def start_monitoring(self):
        """Start background monitoring."""
        self._running = True
        self._ping_thread = threading.Thread(target=self._ping_monitor, daemon=True)
        self._ping_thread.start()
        
    def stop_monitoring(self):
        """Stop monitoring."""
        self._running = False
        
    def record_topic_message(self, topic_name: str):
        """Record that a message was received on a topic."""
        current_time = time.time()
        if topic_name not in self.topic_stats:
            self.topic_stats[topic_name] = []
        
        self.topic_stats[topic_name].append(current_time)
        
        # Keep only last 30 seconds of data
        cutoff = current_time - 30.0
        self.topic_stats[topic_name] = [
            t for t in self.topic_stats[topic_name] if t > cutoff
        ]
        
        self._update_topic_health(topic_name)
        
    def get_network_health(self) -> NetworkHealth:
        """Get comprehensive network health assessment."""
        # Ping metrics
        ping_latency = self._get_avg_ping()
        packet_loss = self._get_packet_loss()
        
        # ROS2 topic health
        ros2_score = self._calculate_ros2_health()
        
        # Overall score (weighted average)
        ping_score = max(0, 100 - ping_latency/10)  # 1000ms = 0 points
        loss_score = max(0, 100 - packet_loss)      # 0% loss = 100 points
        
        overall_score = (ping_score * 0.4 + loss_score * 0.3 + ros2_score * 0.3)
        
        # Determine status
        if overall_score >= 80:
            status = "excellent"
        elif overall_score >= 60:
            status = "good"
        elif overall_score >= 40:
            status = "fair"
        elif overall_score >= 20:
            status = "poor"
        else:
            status = "critical"
            
        return NetworkHealth(
            overall_score=overall_score,
            status=status,
            ping_latency=ping_latency,
            packet_loss=packet_loss,
            ros2_health=ros2_score
        )
    
    def _ping_monitor(self):
        """Background ping monitoring."""
        while self._running:
            try:
                result = subprocess.run(
                    ['ping', '-c', '1', '-W', '2', self.ping_target],
                    capture_output=True, text=True, timeout=5
                )
                
                if result.returncode == 0:
                    # Parse latency from ping output
                    latency_found = False
                    for line in result.stdout.split('\n'):
                        if 'time=' in line:
                            try:
                                # Extract time value (handles both "time=1.23" and "time=1.23 ms")
                                time_part = line.split('time=')[1]
                                latency_str = time_part.split()[0]  # Get first part before space
                                latency = float(latency_str)
                                self._ping_results.append(latency)
                                latency_found = True
                                break
                            except (ValueError, IndexError):
                                continue
                    
                    if not latency_found:
                        # Ping succeeded but couldn't parse latency
                        self._ping_results.append(1000.0)
                else:
                    # Ping failed - record high latency
                    self._ping_results.append(1000.0)
                    
            except subprocess.TimeoutExpired:
                # Ping command timed out
                self._ping_results.append(1000.0)
            except Exception:
                self._ping_results.append(1000.0)
                
            # Keep only last 30 pings
            if len(self._ping_results) > 30:
                self._ping_results = self._ping_results[-30:]
                
            time.sleep(1.0)  # Ping every second
    
    def _update_topic_health(self, topic_name: str):
        """Update health metrics for a specific topic."""
        if topic_name not in self.TOPIC_RATES:
            return
            
        times = self.topic_stats.get(topic_name, [])
        if len(times) < 2:
            return
            
        # Calculate actual rate
        time_span = times[-1] - times[0]
        actual_rate = (len(times) - 1) / time_span if time_span > 0 else 0
        
        # Calculate jitter (rate variance)
        intervals = [times[i] - times[i-1] for i in range(1, len(times))]
        expected_interval = 1.0 / self.TOPIC_RATES[topic_name]
        jitter = sum(abs(interval - expected_interval) for interval in intervals) / len(intervals) if intervals else 0
        
        # Health score based on rate accuracy
        expected_rate = self.TOPIC_RATES[topic_name]
        rate_accuracy = min(actual_rate / expected_rate, expected_rate / actual_rate) if actual_rate > 0 else 0
        health_score = rate_accuracy * 100
        
        self.topic_health[topic_name] = TopicHealth(
            topic_name=topic_name,
            expected_rate=expected_rate,
            actual_rate=actual_rate,
            health_score=health_score,
            last_message_time=times[-1],
            jitter=jitter
        )
    
    def _calculate_ros2_health(self) -> float:
        """Calculate overall ROS2 network health score."""
        if not self.topic_health:
            return 0.0
            
        scores = [topic.health_score for topic in self.topic_health.values()]
        return sum(scores) / len(scores)
    
    def _get_avg_ping(self) -> float:
        """Get average ping latency."""
        return sum(self._ping_results) / len(self._ping_results) if self._ping_results else 1000.0
    
    def _get_packet_loss(self) -> float:
        """Calculate packet loss percentage."""
        if not self._ping_results:
            return 100.0
        failed_pings = sum(1 for ping in self._ping_results if ping >= 1000.0)
        return (failed_pings / len(self._ping_results)) * 100