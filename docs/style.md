# Project Style Guide:
## Commits
As of May 2025, a *conventional commit* strategy will be used.

### Commit structure
Commits will be constructed using the following structure - combining a commit category, scope and message.

**General structure:** `category(scope): Message`

**Example:** A commit for the control systems module that fixes a bug with the PID algorithm.

`fix(control-system): Fix initialisation of PID struct`

### Commit categories
The following outlines the commit categories used (subject to change).

| Commit prefix | Purpose/context                                       |
| ------------- | ----------------------------------------------------- |
| **feat**      | new feature or functionality                          |
| **fix**       | fixing a broken or wrongly implemented feature        |
| **docs**      | updating documentation                                |
| **chore**     | generic/simple change not covered by another category |

### Commit scope
The commit scope is just the submodule that the change belongs to.

For software, this will be the sub-system module (e.g. `control-system`, `gcb` or occasionally `git` for repo changes).

For hardware, this will be the model the commit corresponds to (e.g. `single-axis-jig`, `rpi-desktop-jig`)

## Branching
Branching will be used for any new features or refactors with the following naming convention.

**General structure:** `category/branch-name` (where the category is one of the options described in commit categories)

## C/C++ Code style
- Camel case functions, parameters, members and variables
- Snake case struct names with `_t` suffix
- Snake case enum names with `_e` suffix

Example:
```
// Function style
<type> camelCaseFunctionName(<type> camelCaseParam) {
    // Tabbed indentation
}

// Struct style
typedef struct {
    <type> camelCaseMember;
} custom_struct_t;
```

## Documentation
*Doxygen* documentation will be used with the intent of signalling useful information to the user of each particular file.

For *header files*, documentation will be directed to a caller of the function, leaving out details of implementation but indicating usage.

For *source files*, documentation will be directed to a developer working on or reading the implementation and will therefore contain specific details about complicated or vague implementation.
