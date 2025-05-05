#!/usr/bin/python3
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
from datetime import datetime, timezone

@dataclass
class BlackboardEntry:
    """ข้อมูลที่เก็บใน blackboard พร้อม metadata"""
    value: Any
    timestamp: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    access_count: int = 0

class Blackboard:
    def __init__(self):
        self._data: Dict[str, BlackboardEntry] = {}

    def set(self, key: str, value: Any):
        """เพิ่มหรืออัปเดตคีย์พร้อมค่าใหม่"""
        self._data[key] = BlackboardEntry(value)

    def get(self, key: str, default: Any = None) -> Any:
        """ดึงค่าจาก blackboard พร้อมเพิ่ม access_count"""
        entry = self._data.get(key)
        if entry is None:
            return default
        entry.access_count += 1
        return entry.value

    def pop(self, key: str, default: Any = None) -> Any:
        """ลบและคืนค่าจาก blackboard"""
        entry = self._data.pop(key, None)
        if entry is None:
            return default
        return entry.value

    def get_entry(self, key: str) -> Optional[BlackboardEntry]:
        """คืน BlackboardEntry ทั้งหมดเพื่อดู metadata"""
        return self._data.get(key)

# ตัวอย่างการใช้งาน:
if __name__ == "__main__":
    bb = Blackboard()
    bb.set("target_pos", (1.0, 2.0, 3.0))
    print("Value:", bb.get("target_pos"))
    entry = bb.get_entry("target_pos")
    print("Metadata:", entry)
