---
inclusion: always
---

# No Useless Documentation Files

## Rule: Do Not Create Unnecessary Markdown Files

**NEVER create these types of files:**
- Status/completion logs (e.g., `*_COMPLETE.md`, `*_STATUS.md`, `*_SUMMARY.md`)
- Redundant quick reference guides when main docs exist
- Duplicate checklists or guides
- "Setup complete" announcements
- Progress tracking documents
- Temporary notes or scratch files

## What IS Acceptable

**Only create markdown files that are:**
1. **Essential guides** - Step-by-step instructions for complex procedures
2. **Configuration references** - README files in config directories explaining file purposes
3. **API documentation** - When documenting code interfaces
4. **Architecture docs** - System design and component relationships

## Examples

### ❌ DO NOT CREATE
- `BLUELILY_INTEGRATION_COMPLETE.md` - Status log (useless after completion)
- `ROBOT_HEAD_STATUS.md` - Status log (useless after completion)
- `PHASE1_SETUP_COMPLETE.md` - Announcement (useless)
- `BLUELILY_BRIDGE_AUDIT_COMPLETE.md` - Status log (audit results should be in code comments)
- `BLUELILY_BRIDGE_QUICK_CHECK.md` - Redundant (verification script is enough)
- Multiple checklist files for the same task

### ✅ DO CREATE
- `docs/PHASE1_KALIBR_CALIBRATION.md` - Essential procedure guide
- `kalibr_configs/README.md` - Explains config files in that directory
- `scripts/README.md` - Explains scripts in that directory
- `docs/README.md` - Navigation for documentation

## Rationale

Useless markdown files:
- Clutter the repository
- Become outdated quickly
- Duplicate information
- Add noise without value
- Require maintenance

**Instead:**
- Put status in code comments
- Use git commit messages for completion tracking
- Use scripts with output messages instead of status docs
- Consolidate related information into single comprehensive guides

## When in Doubt

Ask: "Will this file still be useful in 6 months?"
- If it's a status update → NO, don't create it
- If it's a procedure guide → YES, create it
- If it duplicates existing docs → NO, don't create it
- If it's a one-time announcement → NO, don't create it
