## Research Findings for Chapter 1: The Robotic Nervous System (ROS 2)

### RESOLVED: Testing Framework for Code Examples

**Decision**: Use ROS 2's built-in testing infrastructure (`colcon test` with `pytest` for Python nodes)
**Rationale**:
- Native to ROS 2 ecosystem, requiring no additional dependencies
- Pytest is the de facto standard for Python testing, familiar to most developers
- Enables integration testing with ROS 2 launch files and node lifecycle
- Supports both unit tests (individual functions) and integration tests (node communication)
- Educational value: teaches readers industry-standard ROS 2 testing practices
**Alternatives Considered**:
- unittest (Python standard library): Less expressive syntax, less ROS 2 integration
- Manual testing only: No automated validation, poor educational practice
- Robot testing frameworks (e.g., robot_testing): Overkill for basic examples

### RESOLVED: Performance Goals for Code Examples

**Decision**: Educational clarity over performance optimization
**Target Metrics**:
- Message latency: < 50ms for topic communication (acceptable for educational demos)
- Service response time: < 100ms for request/response patterns
- Node startup time: < 2 seconds
- CPU usage: < 10% per node on target hardware (RTX 4070 Ti / Jetson Orin)
- Memory footprint: < 100MB per node
**Rationale**:
- Code examples prioritize readability and conceptual clarity
- Performance metrics ensure examples run smoothly on target hardware
- Goals are appropriate for educational simulation scenarios
- Real-time performance tuning is covered in later advanced chapters
- Metrics provide measurable benchmarks without premature optimization
**Alternatives Considered**:
- Hard real-time constraints: Inappropriate for introductory educational content
- No performance goals: Risk of examples being too slow or resource-intensive
- Production-grade performance: Would require complex optimizations that obscure learning objectives

---

