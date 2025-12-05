# Contract: Sidebar Structure

**Feature**: 001-ai-robotics-textbook
**Date**: 2025-12-05
**Phase**: 1 - Design & Contracts

## Purpose

This contract defines the navigation structure for the Docusaurus sidebar, ensuring consistent organization and easy access to all textbook content.

## Sidebar File: `sidebars.js`

### Complete Structure

```javascript
/**
 * Sidebar configuration for Physical AI & Humanoid Robotics Textbook
 *
 * Structure:
 * - Preface (5 pages)
 * - 8 Parts (27 chapters total + part intros)
 * - Appendices (5 reference documents)
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    // ========================================
    // PREFACE
    // ========================================
    {
      type: 'category',
      label: 'Preface',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'preface/index',
          label: 'What This Textbook Is',
        },
        {
          type: 'doc',
          id: 'preface/how-created',
          label: 'How This Book Was Created',
        },
        {
          type: 'doc',
          id: 'preface/audience',
          label: 'Who This Book Is For',
        },
        {
          type: 'doc',
          id: 'preface/how-to-use',
          label: 'How to Use This Book',
        },
        {
          type: 'doc',
          id: 'preface/requirements',
          label: 'Requirements Overview',
        },
      ],
    },

    // ========================================
    // PART I: FOUNDATIONS
    // ========================================
    {
      type: 'category',
      label: 'Part I: Foundations of Physical AI',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: '01-physical-ai/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: '01-physical-ai/01-introduction',
          label: 'Ch 1: Introduction to Physical AI',
        },
        {
          type: 'doc',
          id: '01-physical-ai/02-humanoid-landscape',
          label: 'Ch 2: Humanoid Robotics Landscape',
        },
        {
          type: 'doc',
          id: '01-physical-ai/03-sensor-foundations',
          label: 'Ch 3: Sensor Foundations',
        },
        {
          type: 'doc',
          id: '01-physical-ai/04-weekly-overview',
          label: 'Ch 4: Course Overview',
        },
      ],
    },

    // ========================================
    // PART II: ROS 2
    // ========================================
    {
      type: 'category',
      label: 'Part II: ROS 2',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: '02-ros2/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: '02-ros2/05-fundamentals',
          label: 'Ch 5: ROS 2 Fundamentals',
        },
        {
          type: 'doc',
          id: '02-ros2/06-python-development',
          label: 'Ch 6: Python Development',
        },
        {
          type: 'doc',
          id: '02-ros2/07-urdf',
          label: 'Ch 7: URDF Robot Description',
        },
        {
          type: 'doc',
          id: '02-ros2/08-control-systems',
          label: 'Ch 8: Control Systems',
        },
      ],
    },

    // ========================================
    // PART III: SIMULATION
    // ========================================
    {
      type: 'category',
      label: 'Part III: Gazebo & Unity',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: '03-gazebo-unity/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: '03-gazebo-unity/09-gazebo-intro',
          label: 'Ch 9: Gazebo Simulation',
        },
        {
          type: 'doc',
          id: '03-gazebo-unity/10-humanoid-sim',
          label: 'Ch 10: Simulating Humanoids',
        },
        {
          type: 'doc',
          id: '03-gazebo-unity/11-unity-robotics',
          label: 'Ch 11: Unity for Robotics',
        },
      ],
    },

    // ========================================
    // PART IV: NVIDIA ISAAC
    // ========================================
    {
      type: 'category',
      label: 'Part IV: NVIDIA Isaac',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: '04-nvidia-isaac/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: '04-nvidia-isaac/12-isaac-sim',
          label: 'Ch 12: Isaac Sim Introduction',
        },
        {
          type: 'doc',
          id: '04-nvidia-isaac/13-perception',
          label: 'Ch 13: Perception & Synthetic Data',
        },
        {
          type: 'doc',
          id: '04-nvidia-isaac/14-isaac-ros',
          label: 'Ch 14: Isaac ROS',
        },
        {
          type: 'doc',
          id: '04-nvidia-isaac/15-navigation',
          label: 'Ch 15: Navigation (Nav2)',
        },
      ],
    },

    // ========================================
    // PART V: VISION-LANGUAGE-ACTION
    // ========================================
    {
      type: 'category',
      label: 'Part V: Vision-Language-Action',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: '05-vla/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: '05-vla/16-vla-intro',
          label: 'Ch 16: VLA Systems Introduction',
        },
        {
          type: 'doc',
          id: '05-vla/17-voice-whisper',
          label: 'Ch 17: Voice with Whisper',
        },
        {
          type: 'doc',
          id: '05-vla/18-llm-planning',
          label: 'Ch 18: LLM Planning',
        },
        {
          type: 'doc',
          id: '05-vla/19-multimodal',
          label: 'Ch 19: Multimodal Interaction',
        },
      ],
    },

    // ========================================
    // PART VI: HARDWARE LAB
    // ========================================
    {
      type: 'category',
      label: 'Part VI: Hardware Lab',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: '06-hardware-lab/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: '06-hardware-lab/20-workstations',
          label: 'Ch 20: Simulation Workstations',
        },
        {
          type: 'doc',
          id: '06-hardware-lab/21-jetson',
          label: 'Ch 21: Jetson Edge AI Kits',
        },
        {
          type: 'doc',
          id: '06-hardware-lab/22-robot-options',
          label: 'Ch 22: Robot Platform Options',
        },
        {
          type: 'doc',
          id: '06-hardware-lab/23-cloud-infra',
          label: 'Ch 23: Cloud Infrastructure',
        },
      ],
    },

    // ========================================
    // PART VII: CAPSTONE
    // ========================================
    {
      type: 'category',
      label: 'Part VII: Capstone Project',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: '07-capstone/index',
          label: 'Part Overview',
        },
        {
          type: 'doc',
          id: '07-capstone/24-architecture',
          label: 'Ch 24: System Architecture',
        },
        {
          type: 'doc',
          id: '07-capstone/25-implementation',
          label: 'Ch 25: Implementation',
        },
        {
          type: 'doc',
          id: '07-capstone/26-sim-to-real',
          label: 'Ch 26: Sim-to-Real Transfer',
        },
        {
          type: 'doc',
          id: '07-capstone/27-evaluation',
          label: 'Ch 27: Evaluation & Extensions',
        },
      ],
    },

    // ========================================
    // PART VIII: APPENDICES
    // ========================================
    {
      type: 'category',
      label: 'Appendices',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: '08-appendix/index',
          label: 'Appendices Overview',
        },
        {
          type: 'doc',
          id: '08-appendix/a-ros2-cheatsheet',
          label: 'A: ROS 2 Cheat Sheets',
        },
        {
          type: 'doc',
          id: '08-appendix/b-urdf-reference',
          label: 'B: URDF/SDF Reference',
        },
        {
          type: 'doc',
          id: '08-appendix/c-troubleshooting',
          label: 'C: Troubleshooting Guide',
        },
        {
          type: 'doc',
          id: '08-appendix/d-hardware-checklists',
          label: 'D: Hardware Checklists',
        },
        {
          type: 'doc',
          id: '08-appendix/e-instructor-guides',
          label: 'E: Instructor Guides',
        },
      ],
    },
  ],
};

module.exports = sidebars;
```

## Structure Requirements

### Hierarchy Levels

1. **Top Level**: Preface, Parts I-VIII, Appendices (10 categories)
2. **Category Level**: Part overviews and chapters within each part
3. **Document Level**: Individual markdown pages

### Sidebar Item Types

#### Category

Used for parts and major sections:

```javascript
{
  type: 'category',
  label: 'Display Name',
  collapsible: true,
  collapsed: false,  // or true
  items: [
    // nested items
  ],
}
```

**Fields**:
- `type`: Always `'category'`
- `label`: Display name in sidebar
- `collapsible`: Allow expand/collapse (always `true`)
- `collapsed`: Initial state (`false` for Preface & Part I, `true` for others)
- `items`: Array of child items

#### Document

Used for individual pages:

```javascript
{
  type: 'doc',
  id: 'folder/filename',
  label: 'Display Name',
}
```

**Fields**:
- `type`: Always `'doc'`
- `id`: Path to markdown file (without `.md` extension, relative to `docs/`)
- `label`: Display name in sidebar

### Naming Conventions

#### Category Labels

| Level | Pattern | Example |
|-------|---------|---------|
| Preface | `"Preface"` | `"Preface"` |
| Part | `"Part N: Title"` | `"Part II: ROS 2"` |
| Appendices | `"Appendices"` | `"Appendices"` |

#### Document Labels

| Type | Pattern | Example |
|------|---------|---------|
| Part overview | `"Part Overview"` | `"Part Overview"` |
| Chapter | `"Ch N: Title"` | `"Ch 5: ROS 2 Fundamentals"` |
| Preface page | `"Descriptive Title"` | `"What This Textbook Is"` |
| Appendix | `"X: Title"` | `"A: ROS 2 Cheat Sheets"` |

### Document ID Patterns

| Type | Pattern | Example |
|------|---------|---------|
| Preface | `preface/filename` | `preface/index` |
| Part overview | `NN-part-name/index` | `02-ros2/index` |
| Chapter | `NN-part-name/NN-chapter-name` | `02-ros2/05-fundamentals` |
| Appendix | `08-appendix/X-name` | `08-appendix/a-ros2-cheatsheet` |

**Rules**:
- No `.md` extension in IDs
- Use kebab-case for filenames
- Match folder structure exactly
- Part folders: `NN-short-name` (01-08)
- Chapter files: `NN-descriptive-name` (01-27)

### Collapsible State Strategy

**Initially Expanded** (`collapsed: false`):
- Preface (students see immediately)
- Part I: Foundations (starting point)

**Initially Collapsed** (`collapsed: true`):
- Parts II-VII (reduce clutter, students expand as needed)
- Appendices (reference material)

**Rationale**:
- Reduces cognitive load on first visit
- Highlights starting content
- Still accessible with one click

## Sidebar Validation Rules

### Structural Requirements

1. **Exactly 10 top-level categories**:
   - 1 Preface
   - 8 Parts
   - 1 Appendices

2. **Part I-VIII must include**:
   - Part overview (`index.md`)
   - All assigned chapters in order

3. **Chapter count verification**:
   - Part I: 4 chapters (1-4)
   - Part II: 4 chapters (5-8)
   - Part III: 3 chapters (9-11)
   - Part IV: 4 chapters (12-15)
   - Part V: 4 chapters (16-19)
   - Part VI: 4 chapters (20-23)
   - Part VII: 4 chapters (24-27)
   - **Total: 27 chapters**

4. **Appendices must include**:
   - Overview page
   - 5 appendices (A-E)

### Ordering Requirements

1. **Preface**: Always first
2. **Parts**: Roman numerals I-VIII in order
3. **Chapters within parts**: Numerical order
4. **Appendices**: Always last

### Document ID Validation

All document IDs must:
- Exist as actual markdown files in `docs/`
- Match folder structure exactly
- Use kebab-case
- Contain no spaces or special characters

### Label Validation

All labels must:
- Be concise (< 50 characters)
- Use title case
- Include chapter numbers for chapters
- Be unique within their category

## Navigation Behavior

### Breadcrumbs

Docusaurus automatically generates breadcrumbs:

```
Home > Part II: ROS 2 > Ch 5: ROS 2 Fundamentals
```

### Previous/Next Navigation

Docusaurus automatically adds previous/next links at the bottom of each page based on sidebar order.

**Example**:
- On "Ch 5: ROS 2 Fundamentals":
  - ← Previous: "Part Overview"
  - Next: "Ch 6: Python Development" →

### Active State

Current page is highlighted in sidebar with accent color.

## Accessibility Requirements

### Keyboard Navigation

Sidebar must be fully navigable via keyboard:
- `Tab`: Move between items
- `Enter/Space`: Expand/collapse categories
- `Arrow keys`: Navigate within categories

### Screen Reader Support

All labels must be descriptive:
- ✅ Good: "Ch 5: ROS 2 Fundamentals"
- ❌ Bad: "Chapter 5" (not descriptive enough)

### ARIA Labels

Docusaurus handles ARIA labels automatically:
- `aria-expanded`: For collapsible categories
- `aria-current`: For active page
- `role="navigation"`: For sidebar container

## Customization Options

### Icons (Optional Future Enhancement)

Add icons to categories:

```javascript
{
  type: 'category',
  label: 'Part II: ROS 2',
  collapsible: true,
  collapsed: true,
  className: 'sidebar-icon-ros2',  // Custom CSS class
  items: [...]
}
```

Then in `custom.css`:
```css
.sidebar-icon-ros2::before {
  content: '🤖';
  margin-right: 0.5rem;
}
```

### Custom Categories (If Needed)

For cross-cutting topics (not implemented yet, but possible):

```javascript
{
  type: 'category',
  label: '🔧 Quick References',
  collapsible: true,
  collapsed: true,
  items: [
    {type: 'ref', id: '08-appendix/a-ros2-cheatsheet'},
    {type: 'ref', id: '08-appendix/b-urdf-reference'},
  ],
}
```

## Maintenance Guidelines

### Adding a New Chapter

1. Create markdown file in appropriate `docs/NN-part/` folder
2. Add frontmatter with `sidebar_position`
3. Add entry to `sidebars.js` in correct position
4. Verify chapter numbering is sequential
5. Test navigation locally

### Reordering Chapters

1. Update `sidebar_position` in chapter frontmatter
2. Update `sidebars.js` order
3. Update chapter numbers in labels if needed
4. Update cross-references in other chapters

### Adding a New Part

1. Create folder: `docs/NN-new-part/`
2. Create `index.md` for part overview
3. Add chapters with sequential numbering
4. Add category to `sidebars.js` in correct position
5. Update total chapter count validation

## Testing Checklist

Before deploying sidebar changes:

- [ ] All document IDs match actual file paths
- [ ] Chapter count = 27
- [ ] Chapter numbers sequential (1-27)
- [ ] Part numbers sequential (I-VIII)
- [ ] Appendix letters sequential (A-E)
- [ ] All labels descriptive and < 50 chars
- [ ] Preface and Part I initially expanded
- [ ] Other parts initially collapsed
- [ ] Breadcrumbs work correctly
- [ ] Previous/Next navigation logical
- [ ] Active page highlights correctly
- [ ] Keyboard navigation works
- [ ] Mobile responsive (test on phone)
- [ ] No JavaScript errors in console

## Performance Considerations

### Sidebar Size

With 27 chapters + preface + appendices + part overviews:
- **Total items**: ~45 documents
- **Performance impact**: Minimal (Docusaurus handles efficiently)

### Lazy Loading

Docusaurus lazy loads document content, not sidebar items.
All sidebar items load immediately for instant navigation.

### Bundle Size

Sidebar configuration adds ~2KB to bundle (negligible).

## Migration Path

### From Development to Production

1. **Initial Setup**:
   ```bash
   # Start development server
   npm start
   # Sidebar hot-reloads automatically
   ```

2. **Validate Structure**:
   - Check all links work
   - Verify chapter order
   - Test expand/collapse

3. **Deploy**:
   ```bash
   npm run build
   # Sidebar baked into production build
   ```

### Updating Sidebar

When adding content:

1. Edit `sidebars.js`
2. Add new markdown files
3. Test locally with `npm start`
4. Commit changes
5. Deploy via CI/CD

## Integration with Other Systems

### Search Integration

Sidebar structure feeds into search:
- Categories become search facets
- Labels become search result titles
- Hierarchy aids result ranking

### Sitemap Generation

Docusaurus generates `sitemap.xml` from sidebar:
```xml
<url>
  <loc>https://domain.com/02-ros2/05-fundamentals</loc>
  <changefreq>monthly</changefreq>
  <priority>0.8</priority>
</url>
```

### Analytics

Track sidebar navigation:
- Most expanded categories
- Chapter access patterns
- Drop-off points

## References

- [Docusaurus Sidebar Documentation](https://docusaurus.io/docs/sidebar)
- [Sidebar Item Types](https://docusaurus.io/docs/sidebar/items)
- [Auto-Generated Sidebars](https://docusaurus.io/docs/sidebar/autogenerated)
- [Sidebar Customization](https://docusaurus.io/docs/sidebar/multiple-sidebars)
