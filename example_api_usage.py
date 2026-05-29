#!/usr/bin/env python3
"""
Example script: Testing PC2 UR5 API from PC1.
Shows how to:
  1. Send job to PC2
  2. Poll status
  3. Handle results
  4. Abort if needed
"""

import requests
import json
import time
import sys


# PC2 server address
PC2_URL = "http://localhost:5001"


def health_check() -> bool:
    """Check if PC2 is online."""
    try:
        response = requests.get(f"{PC2_URL}/api/ur5/health", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"✓ PC2 Online: {data}")
            return True
        else:
            print(f"✗ PC2 unhealthy: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ PC2 unreachable: {e}")
        return False


def submit_job(station: str, workflow_id: str) -> str:
    """Submit job to PC2, return job_id."""
    payload = {
        "station": station,
        "workflow_id": workflow_id
    }

    print(f"\n📤 Submitting job: {payload}")

    response = requests.post(
        f"{PC2_URL}/api/ur5/execute",
        json=payload,
        timeout=5
    )

    if response.status_code == 202:
        data = response.json()
        job_id = data["job_id"]
        print(f"✓ Job accepted: {job_id}")
        return job_id
    elif response.status_code == 409:
        print(f"✗ Conflict: Another job is already running")
        data = response.json()
        print(f"   Active job: {data.get('active_job_id')}")
        return None
    else:
        print(f"✗ Failed: {response.status_code} - {response.text}")
        return None


def get_status(job_id: str) -> dict:
    """Get job status."""
    response = requests.get(
        f"{PC2_URL}/api/ur5/status/{job_id}",
        timeout=5
    )

    if response.status_code == 200:
        return response.json()
    elif response.status_code == 404:
        print(f"✗ Job {job_id} not found")
        return None
    else:
        print(f"✗ Error: {response.status_code}")
        return None


def poll_job(job_id: str, max_wait_seconds: int = 300, poll_interval: int = 5) -> dict:
    """Poll job status until done."""
    print(f"\n⏳ Polling job {job_id}...")
    
    start_time = time.time()
    
    while time.time() - start_time < max_wait_seconds:
        job = get_status(job_id)
        
        if job is None:
            return None
        
        status = job.get("status")
        phase = job.get("phase")
        parts_found = job.get("parts_found")
        parts_picked = job.get("parts_picked")
        
        elapsed = time.time() - start_time
        print(f"  [{elapsed:6.1f}s] Status: {status:10s} | Phase: {phase:30s} | "
              f"Found: {parts_found} | Picked: {parts_picked}")
        
        if status == "done":
            print(f"✓ Job completed successfully")
            return job
        elif status in ["error", "aborted"]:
            print(f"✗ Job {status}: {job.get('error')}")
            return job
        
        time.sleep(poll_interval)
    
    print(f"✗ Timeout waiting for job (max {max_wait_seconds}s)")
    return job


def abort_job(job_id: str) -> bool:
    """Request job abort."""
    print(f"\n🛑 Requesting abort for job {job_id}...")
    
    response = requests.post(
        f"{PC2_URL}/api/ur5/abort/{job_id}",
        timeout=5
    )
    
    if response.status_code == 200:
        print(f"✓ Abort requested")
        return True
    elif response.status_code == 404:
        print(f"✗ Job not found")
        return False
    else:
        print(f"✗ Error: {response.status_code}")
        return False


def print_job_details(job: dict) -> None:
    """Pretty print job details."""
    if not job:
        return
    
    print(f"\n📊 Job Details:")
    print(f"  ID:           {job['job_id']}")
    print(f"  Status:       {job['status']}")
    print(f"  Phase:        {job['phase']}")
    print(f"  Station:      {job['station']}")
    print(f"  Workflow ID:  {job['workflow_id']}")
    print(f"  Parts Found:  {job['parts_found']}")
    print(f"  Parts Picked: {job['parts_picked']}")
    print(f"  Error:        {job['error'] or 'None'}")
    print(f"  Created:      {job['created_at']}")
    print(f"  Updated:      {job['updated_at']}")
    
    if job.get('log'):
        print(f"\n📝 Last 10 log entries:")
        for entry in job['log'][-10:]:
            print(f"    {entry}")


def main():
    """Main example flow."""
    print("=" * 70)
    print("PC2 UR5 Control - API Example")
    print("=" * 70)
    
    # 1. Health check
    if not health_check():
        print("\n❌ PC2 is not responding. Start server with: python app.py")
        sys.exit(1)
    
    # 2. Submit job
    job_id = submit_job(
        station="khay_test_01",
        workflow_id="workflow_12345"
    )
    
    if not job_id:
        print("\n❌ Failed to submit job")
        sys.exit(1)
    
    # 3. Poll status
    final_job = poll_job(job_id, max_wait_seconds=300, poll_interval=2)
    
    if final_job:
        print_job_details(final_job)
        
        # 4. Summary
        print("\n" + "=" * 70)
        if final_job['status'] == 'done':
            print(f"✅ SUCCESS: Picked {final_job['parts_picked']}/{final_job['parts_found']} parts")
        else:
            print(f"❌ FAILED: {final_job['status']} - {final_job['error']}")
        print("=" * 70)


def example_abort():
    """Example: Abort a running job."""
    print("=" * 70)
    print("Example: Abort Job")
    print("=" * 70)
    
    # Submit a job
    job_id = submit_job("khay_test_02", "workflow_abort_test")
    if not job_id:
        return
    
    # Wait 5 seconds
    print("\nWaiting 5s before abort...")
    time.sleep(5)
    
    # Abort
    abort_job(job_id)
    
    # Poll to see abort progress
    for _ in range(30):
        job = get_status(job_id)
        if job and job['status'] in ['aborted', 'error', 'done']:
            print_job_details(job)
            break
        time.sleep(1)


def example_concurrent_job_handling():
    """Example: Try to submit second job while one is running."""
    print("=" * 70)
    print("Example: Concurrent Job Handling")
    print("=" * 70)
    
    # Submit first job
    job1 = submit_job("khay_1", "workflow_1")
    if not job1:
        return
    
    print("\nFirst job submitted. Trying to submit second job...")
    time.sleep(1)
    
    # Try to submit second job (should fail with 409)
    job2 = submit_job("khay_2", "workflow_2")
    if job2 is None:
        print("✓ Second job correctly rejected (409 Conflict)")
    
    # Wait for first job
    print("\nWaiting for first job...")
    time.sleep(5)
    
    # Now submit second job (should work)
    print("\nTrying to submit second job again...")
    job2 = submit_job("khay_2", "workflow_2")
    if job2:
        print("✓ Second job accepted")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="PC2 UR5 API Example")
    parser.add_argument(
        "--example",
        choices=["basic", "abort", "concurrent"],
        default="basic",
        help="Which example to run"
    )
    parser.add_argument(
        "--pc2-url",
        default="http://localhost:5001",
        help="PC2 server URL"
    )
    
    args = parser.parse_args()
    PC2_URL = args.pc2_url
    
    if args.example == "basic":
        main()
    elif args.example == "abort":
        example_abort()
    elif args.example == "concurrent":
        example_concurrent_job_handling()
