*The following is a template for Pull Requests. Please fill the following sections out when you submit a PR.*

# What this does
- What bug does this PR fix?
- What feature does it implement?
- Links to design document / snippet

# Testing
What testing has been done on this PR? Show proof here that this PR doesn't cause any regressions and improves performance in the manner it is expected to. Post pictures, graphs, logs, etc here.

Examples of testing often done prior to PR:
- For measurement engine changes:
  - libswiftnav unit tests
  - simulation in Peregrine
  - bench testing (with simulator or roof)
- For positioning changes:
  - libswiftnav unit tests
  - gnss_analysis test suite

## Testing Checklist
If these checks are not required, explain why. For HITL tests, see the Travis comment that will appear on this PR for link to HITL results.
- [ ] Has this PR been HITL tested and has it passed all passfail checks?
- [ ] Has this PR been HITL tested and has piksi-multi-PRD performance not regressed?
